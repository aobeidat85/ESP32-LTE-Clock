// SPDX-License-Identifier: GPL-3.0-only
// Bounded C++ port of the experiment's LTE back end. Turbo algorithm adapted
// from Daniel Estevez's MIT implementation; see THIRD_PARTY_NOTICES.md.
#include "LteTimeDecoder.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace esp32_lte_time {
namespace {
constexpr float Neg = -1.0e30f;
float clip(float x) { return std::max(-512.0f, std::min(512.0f, x)); }
float addLog(float a, float b) {
  if (a <= Neg / 2) return b;
  if (b <= Neg / 2) return a;
  float delta = std::fabs(a - b);
  return std::max(a, b) + (delta > 16 ? 0 : std::log1p(std::exp(-delta)));
}
struct SchemaNode {
  uint8_t kind, extensible, optional, tag;
  uint16_t first, count;
  int32_t lower, upper;
};
#include "LteTimeTables.inc"
constexpr uint8_t TurboPerm[32] = {
  0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30,
  1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31
};
constexpr uint8_t ConvPerm[32] = {
  1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31,
  0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30
};

unsigned width(uint64_t range) {
  unsigned n = 0;
  while (range) { ++n; range >>= 1; }
  return n;
}
struct BitReader {
  const uint8_t* data;
  size_t size, pos = 0;
  bool good = true;
  BitReader(const uint8_t* bytes, size_t bits) : data(bytes), size(bits) {}
  uint64_t get(unsigned n) {
    if (n > 64 || pos > size || n > size - pos) { good = false; return 0; }
    uint64_t v = 0;
    while (n--) { v = (v << 1) | ((data[pos / 8] >> (7 - pos % 8)) & 1); ++pos; }
    return v;
  }
  uint32_t constrained(int32_t lo, int32_t hi) {
    uint32_t range = uint32_t(int64_t(hi) - lo);
    uint32_t x = uint32_t(get(width(range)));
    if (x > range) good = false;
    return x; // Offset from lower bound, also handles negative INTEGER bounds.
  }
  bool skip(size_t n) {
    if (pos > size || n > size - pos) return good = false;
    pos += n; return good;
  }
  uint32_t small() {
    if (get(1)) { good = false; return 0; } // Large extension maps unsupported.
    return uint32_t(get(6));
  }
  bool openType() {
    uint32_t n = uint32_t(get(8));
    if ((n & 0xc0) == 0xc0) return good = false; // Fragmentation unsupported.
    if (n & 0x80) n = ((n & 0x3f) << 8) | uint32_t(get(8));
    return good && skip(size_t(n) * 8);
  }
};

bool readNode(BitReader& b, unsigned index, Sib8Time& time, unsigned& found, unsigned depth = 0) {
  if (!b.good || depth > 24 || index >= sizeof(schema) / sizeof(schema[0])) return false;
  const auto& n = schema[index];
  bool extension = n.extensible && b.get(1);
  if (n.kind == 0) { // SEQUENCE: extension flag, root optional bitmap, components.
    bool present[16] = {};
    if (n.count > 16) return false;
    for (unsigned i = 0; i < n.count; ++i)
      present[i] = schema[children[n.first + i]].optional ? b.get(1) : true;
    for (unsigned i = 0; i < n.count; ++i)
      if (present[i] && !readNode(b, children[n.first + i], time, found, depth + 1)) return false;
    if (extension) {
      unsigned count = b.small() + 1;
      if (count > 64 || !b.good) return false;
      uint64_t bitmap = b.get(count);
      for (unsigned i = 0; i < count; ++i)
        if ((bitmap >> (count - i - 1)) & 1) if (!b.openType()) return false;
    }
  } else if (n.kind == 1) { // CHOICE
    if (extension) { b.small(); return b.good && b.openType(); }
    unsigned selected = b.constrained(0, n.count - 1);
    if (!b.good || selected >= n.count) return false;
    return readNode(b, children[n.first + selected], time, found, depth + 1);
  } else if (n.kind == 2) { // SEQUENCE OF
    if (extension) return false;
    unsigned count = b.constrained(n.lower, n.upper) + n.lower;
    for (unsigned i = 0; i < count && b.good; ++i)
      if (!readNode(b, children[n.first], time, found, depth + 1)) return false;
  } else if (n.kind == 3 || n.kind == 4) { // INTEGER / ENUMERATED
    if (extension) b.small();
    else b.constrained(n.lower, n.upper);
  } else if (n.kind == 5) { // BOOLEAN
    bool value = b.get(1);
    if (n.tag == 1) time.eutraSynchronized = value;
  } else if (n.kind == 6) { // BIT STRING
    if (extension) return false;
    unsigned count = b.constrained(n.lower, n.upper) + n.lower;
    if (n.tag == 2 || n.tag == 3) {
      if (++found != 1 || count > 64) return false;
      time.ticks = b.get(count); time.width = uint8_t(count);
    } else b.skip(count);
  } else return false;
  return b.good;
}

uint16_t crc16(const uint8_t* bits, size_t count) {
  uint16_t reg = 0;
  for (size_t i = 0; i < count; ++i) {
    bool top = (reg >> 15) ^ bits[i];
    reg <<= 1;
    if (top) reg ^= 0x1021;
  }
  return reg;
}
unsigned parityBit(unsigned value) {
  value ^= value >> 4; value ^= value >> 2; value ^= value >> 1;
  return value & 1;
}
} // namespace

const char* resultName(Result r) {
  switch (r) {
    case Result::Ok: return "ok";
    case Result::InvalidInput: return "invalid input";
    case Result::Unsupported: return "unsupported profile";
    case Result::CrcFailed: return "CRC failed";
    case Result::MalformedRrc: return "malformed RRC";
    case Result::NoTime: return "no SIB8 time";
    case Result::LeapTableExpired: return "leap table expired";
    case Result::LeapSecond: return "unrepresentable POSIX leap second";
    case Result::InvalidReference: return "invalid or stale device reference";
  }
  return "unknown";
}

uint32_t crc24a(const uint8_t* data, size_t bytes) {
  uint32_t reg = 0;
  for (size_t i = 0; i < bytes; ++i) {
    reg ^= uint32_t(data[i]) << 16;
    for (unsigned j = 0; j < 8; ++j) {
      reg <<= 1;
      if (reg & 0x1000000) reg ^= 0x1864cfb;
    }
  }
  return reg & 0xffffff;
}

float TransportDecoder::systematic(unsigned branch, size_t t) const {
  if (t < k_) return channel_[0][branch ? permutation_[t] : t];
  const unsigned row[2][3] = {{0,2,1}, {0,2,1}};
  const unsigned col[2][3] = {{0,0,1}, {2,2,3}};
  return channel_[row[branch][t - k_]][k_ + col[branch][t - k_]];
}
float TransportDecoder::parity(unsigned branch, size_t t) const {
  if (t < k_) return channel_[branch ? 2 : 1][t];
  const unsigned row[3] = {1,0,2};
  const unsigned col[2][3] = {{0,1,1}, {2,3,3}};
  return channel_[row[t - k_]][k_ + col[branch][t - k_]];
}

void TransportDecoder::bcjr(unsigned branch) {
  const size_t steps = k_ + 3;
  auto gamma = [&](size_t t, unsigned state, unsigned input) {
    unsigned feedback = (state ^ (state >> 1)) & 1;
    unsigned bit = t < k_ ? input : feedback;
    unsigned next = ((state >> 1) | ((bit ^ feedback) << 2));
    unsigned p = (state ^ (state >> 2) ^ (next >> 2)) & 1;
    float value = .5f * ((bit ? -1.f : 1.f) * (systematic(branch, t) + (t < k_ ? prior_[t] : 0))
                       + (p ? -1.f : 1.f) * parity(branch, t));
    return value;
  };
  for (unsigned s = 0; s < 8; ++s) alpha_[0][s] = s == 0 ? 0 : Neg;
  for (size_t t = 0; t < steps; ++t) {
    if (service_ && t % 64 == 0) service_();
    std::fill(alpha_[t + 1], alpha_[t + 1] + 8, Neg);
    for (unsigned s = 0; s < 8; ++s) for (unsigned bit = 0; bit < (t < k_ ? 2u : 1u); ++bit) {
      unsigned feedback = (s ^ (s >> 1)) & 1;
      unsigned next = (s >> 1) | ((t < k_ ? bit ^ feedback : 0) << 2);
      alpha_[t + 1][next] = addLog(alpha_[t + 1][next], alpha_[t][s] + gamma(t, s, bit));
    }
    float maximum = *std::max_element(alpha_[t + 1], alpha_[t + 1] + 8);
    for (float& value : alpha_[t + 1]) value -= maximum;
  }
  float beta[8] = {0,Neg,Neg,Neg,Neg,Neg,Neg,Neg};
  for (size_t t = steps; t-- > 0;) {
    if (service_ && t % 64 == 0) service_();
    float previous[8], hypotheses[2] = {Neg, Neg};
    for (unsigned s = 0; s < 8; ++s) {
      previous[s] = Neg;
      for (unsigned bit = 0; bit < (t < k_ ? 2u : 1u); ++bit) {
        unsigned feedback = (s ^ (s >> 1)) & 1;
        unsigned next = (s >> 1) | ((t < k_ ? bit ^ feedback : 0) << 2);
        float value = gamma(t, s, bit) + beta[next];
        previous[s] = addLog(previous[s], value);
        if (t < k_) hypotheses[bit] = addLog(hypotheses[bit], alpha_[t][s] + value);
      }
    }
    if (t < k_) posterior_[t] = hypotheses[0] - hypotheses[1];
    float maximum = *std::max_element(previous, previous + 8);
    for (unsigned s = 0; s < 8; ++s) beta[s] = previous[s] - maximum;
  }
}

Result TransportDecoder::decode(const float* llr, size_t count, unsigned rv, size_t transportBits,
                                uint8_t* payload, size_t capacity, unsigned iterations,
                                ServiceCallback service) {
  if (!llr || !payload || !count || count > 32768 || rv > 3 || transportBits % 8 ||
      !transportBits || !iterations || iterations > 16)
    return Result::InvalidInput;
  if (transportBits > MaxTransportBits) return Result::Unsupported;
  if (capacity < transportBits / 8) return Result::InvalidInput;
  Result r = begin(transportBits, rv);
  if (r != Result::Ok) return r;
  for (size_t i = 0; i < count; ++i) if ((r = push(llr[i])) != Result::Ok) return r;
  return finish(payload, capacity, iterations, service);
}

Result TransportDecoder::begin(size_t transportBits, unsigned rv) {
  inputStatus_ = Result::InvalidInput;
  if (!transportBits || transportBits % 8 || rv > 3) return inputStatus_;
  if (transportBits > MaxTransportBits) return inputStatus_ = Result::Unsupported;
  k_ = transportBits + 24;
  unsigned f1 = 0, f2 = 0;
  for (const auto& row : qpp) if (row[0] == k_) { f1 = row[1]; f2 = row[2]; break; }
  if (!f1) return inputStatus_ = Result::Unsupported;
  for (size_t i = 0; i < k_; ++i) permutation_[i] = uint16_t((f1 * i + f2 * i * i) % k_);
  std::memset(channel_, 0, sizeof(channel_));
  const size_t rows = (k_ + 4 + 31) / 32;
  padded_ = rows * 32;
  cursor_ = rows * (2 * ((3 * padded_ + 8 * rows - 1) / (8 * rows)) * rv + 2);
  seen_ = 0; energy_ = 0;
  return inputStatus_ = Result::Ok;
}

Result TransportDecoder::push(float llr) {
  if (inputStatus_ != Result::Ok) return inputStatus_;
  if (++seen_ > 32768 || !std::isfinite(llr) || std::fabs(llr) > 1e6f)
    return inputStatus_ = Result::InvalidInput;
  energy_ += std::fabs(llr);
  const size_t rows = padded_ / 32, pad = padded_ - (k_ + 4);
    for (;;) {
      size_t position = cursor_++ % (3 * padded_);
      unsigned branch = position < padded_ ? 0 : 1 + (position - padded_) % 2;
      size_t j = position < padded_ ? position : (position - padded_) / 2;
      size_t source = TurboPerm[j / rows] + 32 * (j % rows);
      if (branch == 2) source = (source + 1) % padded_;
      if (source < pad) continue;
      channel_[branch][source - pad] += llr; break;
    }
  return Result::Ok;
}

Result TransportDecoder::finish(uint8_t* payload, size_t capacity, unsigned iterations, ServiceCallback service) {
  if (inputStatus_ != Result::Ok) return inputStatus_;
  if (!payload || capacity < (k_ - 24) / 8 || !iterations || iterations > 16) return Result::InvalidInput;
  inputStatus_ = Result::InvalidInput; // A block can be finalized only once.
  service_ = service;
  if (!seen_ || energy_ < 1e-6f) return Result::CrcFailed;
  for (auto& row : channel_) for (float& v : row) v = clip(v);
  std::fill(extrinsic_, extrinsic_ + k_, 0.0f);
  uint8_t candidate[MaxK / 8] = {};
  for (unsigned iteration = 0; iteration < iterations; ++iteration) {
    std::copy(extrinsic_, extrinsic_ + k_, prior_);
    bcjr(0);
    for (size_t t = 0; t < k_; ++t) {
      unsigned p = permutation_[t];
      prior_[t] = clip(posterior_[p] - channel_[0][p] - extrinsic_[p]);
    }
    bcjr(1);
    std::memset(candidate, 0, sizeof(candidate));
    for (size_t t = 0; t < k_; ++t) {
      unsigned p = permutation_[t];
      extrinsic_[p] = clip(posterior_[t] - systematic(1, t) - prior_[t]);
      if (posterior_[t] < 0) candidate[p / 8] |= 1u << (7 - p % 8);
    }
    if (!crc24a(candidate, k_ / 8)) {
      std::memcpy(payload, candidate, (k_ - 24) / 8); return Result::Ok;
    }
  }
  return Result::CrcFailed;
}

Result parseSib8(const uint8_t* payload, size_t bytes, Sib8Time& time) {
  time = {};
  if (!payload || !bytes || bytes > MaxTransportBits / 8) return Result::InvalidInput;
  BitReader b{payload, bytes * 8};
  if (b.get(1) || b.get(1) || b.get(1)) return Result::Unsupported;
  bool nonCriticalExtension = b.get(1);
  unsigned count = unsigned(b.get(5)) + 1, found = 0;
  Sib8Time candidate;
  for (unsigned i = 0; i < count && b.good; ++i) {
    if (b.get(1)) return Result::Unsupported; // Extension SIB CHOICE.
    unsigned choice = unsigned(b.get(4));
    if (choice != 3 && choice != 6) return Result::Unsupported;
    if (!readNode(b, choice == 3 ? Sib5Node : Sib8Node, candidate, found)) return Result::MalformedRrc;
  }
  if (nonCriticalExtension) return Result::Unsupported;
  // BCCH can be zero-padded to the scheduled transport-block size.
  if (!b.good) return Result::MalformedRrc;
  while (b.pos < b.size)
    if (b.get(unsigned(std::min(size_t(64), b.size - b.pos)))) return Result::MalformedRrc;
  if (found != 1) return Result::NoTime;
  time = candidate; return Result::Ok;
}

Result toUtc(const Sib8Time& time, UtcTime& utc) {
  utc = {};
  if ((time.width != 39 && time.width != 49) || time.ticks >= (uint64_t(1) << time.width))
    return Result::InvalidInput;
  const uint64_t units = time.width == 39 ? 100 : 153600;
  int64_t gpsUs = 315964800000000LL + int64_t(time.ticks / units) * 1000000
                + int64_t(((time.ticks % units) * 1000000 + units / 2) / units);
  for (size_t i = 1; i < sizeof(leaps) / sizeof(leaps[0]); ++i) {
    if (leaps[i].offset <= leaps[i - 1].offset) continue;
    int64_t first = (leaps[i].utc + leaps[i - 1].offset) * 1000000;
    int64_t last = (leaps[i].utc + leaps[i].offset) * 1000000;
    if (gpsUs >= first && gpsUs < last) return Result::LeapSecond;
  }
  for (size_t i = sizeof(leaps) / sizeof(leaps[0]); i-- > 0;) {
    int64_t candidate = gpsUs - int64_t(leaps[i].offset) * 1000000;
    if (candidate < leaps[i].utc * 1000000) continue;
    if (candidate >= LeapExpiry * 1000000) return Result::LeapTableExpired;
    utc.unixUs = candidate; utc.gpsMinusUtc = leaps[i].offset;
    utc.resolutionNs = time.width == 39 ? 10000000 : 6511;
    return Result::Ok;
  }
  return Result::InvalidInput;
}

Result advanceToNow(const UtcTime& atBoundary, int64_t boundaryUptimeUs,
                    int64_t nowUptimeUs, uint64_t maxAgeUs, int64_t& unixUs) {
  if (atBoundary.unixUs <= 0 || boundaryUptimeUs < 0 || nowUptimeUs < boundaryUptimeUs)
    return Result::InvalidReference;
  int64_t age = nowUptimeUs - boundaryUptimeUs;
  if (uint64_t(age) > maxAgeUs || atBoundary.unixUs > std::numeric_limits<int64_t>::max() - age)
    return Result::InvalidReference;
  unixUs = atBoundary.unixUs + age;
  return Result::Ok;
}

Result decodeSiControl(const float* llr, size_t count, size_t payloadBits,
                       uint8_t* bits, size_t capacity, ServiceCallback service, uint16_t crcMask, bool fastOnly) {
  if (!llr || !bits || !count || count > 576 || !payloadBits || payloadBits > 48 || capacity < payloadBits)
    return Result::InvalidInput;
  const size_t d = payloadBits + 16, rows = (d + 31) / 32, pad = rows * 32 - d;
  uint16_t map[192]; size_t used = 0;
  for (unsigned branch = 0; branch < 3; ++branch)
    for (unsigned col = 0; col < 32; ++col) for (unsigned row = 0; row < rows; ++row) {
      unsigned source = 32 * row + ConvPerm[col];
      if (source >= pad) map[used++] = uint16_t(branch * d + source - pad);
    }
  float sums[192] = {}, energy = 0;
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(llr[i]) || std::fabs(llr[i]) > 1e6f) return Result::InvalidInput;
    sums[map[i % used]] += llr[i]; energy += std::fabs(llr[i]);
  }
  if (energy < 1e-6f) return Result::CrcFailed;
  if (fastOnly) {
    // Circular Viterbi candidate search. Repeating the block warms up the
    // unknown start state. A closed traceback AND the broadcast CRC mask are
    // mandatory; uncertain candidates are rejected, never accepted as time.
    float metric[64] = {}, next[64];uint64_t choices[64];uint8_t decoded[64];
    for (unsigned block = 0; block < 4; ++block) {
      if (service) service();
      for (size_t t = 0; t < d; ++t) {
        choices[t] = 0;
        auto branch = [&](unsigned state, unsigned bit) {
          unsigned reg = (state << 1) | bit;
          return (1 - 2 * int(parityBit(reg & 0x6d))) * sums[t]
            + (1 - 2 * int(parityBit(reg & 0x4f))) * sums[d + t]
            + (1 - 2 * int(parityBit(reg & 0x57))) * sums[2 * d + t];
        };
        for (unsigned s = 0; s < 64; ++s) {
          unsigned p = s >> 1, bit = s & 1;
          float a = metric[p] + branch(p, bit), b = metric[p + 32] + branch(p + 32, bit);
          if (b > a) choices[t] |= uint64_t(1) << s;
          next[s] = std::max(a, b);
        }
        std::copy(next, next + 64, metric);
      }
      unsigned end = unsigned(std::max_element(metric, metric + 64) - metric), state = end;
      for (size_t t = d; t-- > 0;) {
        decoded[t] = state & 1;
        state = (state >> 1) + 32 * unsigned((choices[t] >> state) & 1);
      }
      uint16_t received = 0;
      for (size_t i = payloadBits; i < d; ++i) received = (received << 1) | decoded[i];
      if (state == end && uint16_t(crc16(decoded, payloadBits) ^ crcMask) == received) {
        std::memcpy(bits, decoded, payloadBits);return Result::Ok;
      }
      float maximum = metric[end];for (float& value : metric) value -= maximum;
    }
    return Result::CrcFailed;
  }
  float branchMetric[64][2] = {};
  uint64_t choices[64] = {};
  unsigned best = 0; float bestMetric = Neg;
  // Retain the exact closed-path tail-biting search, but only store one path.
  for (unsigned trial = 0; trial < 65; ++trial) {
    if (service) service();
    unsigned initial = trial == 64 ? best : trial;
    float metric[64], next[64];
    std::fill(metric, metric + 64, Neg); metric[initial] = 0;
    for (size_t t = 0; t < d; ++t) {
      for (unsigned s = 0; s < 64; ++s) for (unsigned bit = 0; bit < 2; ++bit) {
        unsigned reg = (s << 1) | bit;
        branchMetric[s][bit] = (1 - 2 * int(parityBit(reg & 0x6d))) * sums[t]
          + (1 - 2 * int(parityBit(reg & 0x4f))) * sums[d + t]
          + (1 - 2 * int(parityBit(reg & 0x57))) * sums[2 * d + t];
      }
      choices[t] = 0;
      for (unsigned s = 0; s < 64; ++s) {
        unsigned p = s >> 1, bit = s & 1;
        float a = metric[p] + branchMetric[p][bit];
        float b = metric[p + 32] + branchMetric[p + 32][bit];
        if (b > a) choices[t] |= uint64_t(1) << s;
        next[s] = std::max(a, b);
      }
      std::copy(next, next + 64, metric);
    }
    if (trial < 64 && metric[initial] > bestMetric) { best = initial; bestMetric = metric[initial]; }
  }
  uint8_t decoded[64]; unsigned state = best;
  for (size_t t = d; t-- > 0;) {
    decoded[t] = state & 1;
    state = (state >> 1) + 32 * unsigned((choices[t] >> state) & 1);
  }
  uint16_t received = 0;
  for (size_t i = payloadBits; i < d; ++i) received = (received << 1) | decoded[i];
  if (state != best || uint16_t(crc16(decoded, payloadBits) ^ crcMask) != received) return Result::CrcFailed;
  std::memcpy(bits, decoded, payloadBits); return Result::Ok;
}
} // namespace esp32_lte_time
