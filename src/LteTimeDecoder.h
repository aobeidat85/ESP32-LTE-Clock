// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <stddef.h>
#include <stdint.h>

// Portable, bounded-memory back end. Does not acquire RF or use the host clock.
namespace esp32_lte_time {
using ServiceCallback = void (*)(); // Optional scheduler/watchdog service.
constexpr size_t MaxTransportBits = 488;  // One LTE code block, K <= 512.
constexpr size_t MaxK = MaxTransportBits + 24;
enum class Result : uint8_t {
  Ok, InvalidInput, Unsupported, CrcFailed, MalformedRrc, NoTime,
  LeapTableExpired, LeapSecond, InvalidReference
};
const char* resultName(Result result);

struct Sib8Time {
  uint64_t ticks = 0;
  uint8_t width = 0;  // 39: 10 ms; 49: 1/153600 second.
  bool eutraSynchronized = false;
};
struct UtcTime {
  int64_t unixUs = 0;             // Time at the broadcast's reference boundary.
  int8_t gpsMinusUtc = 0;
  uint32_t resolutionNs = 0;     // Encoding resolution, not measured accuracy.
};

// Allocate once, outside the ESP32 loop-task stack. Reusable, not thread-safe.
class TransportDecoder {
 public:
  Result decode(const float* llr, size_t count, unsigned rv, size_t transportBits,
                uint8_t* payload, size_t capacity, unsigned iterations = 8,
                ServiceCallback service = nullptr);
  // Streaming input avoids a second full-subframe LLR buffer on the ESP32.
  Result begin(size_t transportBits, unsigned rv);
  Result push(float llr);
  Result finish(uint8_t* payload, size_t capacity, unsigned iterations = 8,
                ServiceCallback service = nullptr);
  static constexpr size_t workspaceBytes();
 private:
  float channel_[3][MaxK + 4];
  float alpha_[MaxK + 4][8];
  float prior_[MaxK], extrinsic_[MaxK], posterior_[MaxK];
  uint16_t permutation_[MaxK];
  size_t k_ = 0;
  ServiceCallback service_ = nullptr;
  size_t cursor_ = 0, padded_ = 0, seen_ = 0;
  float energy_ = 0;
  Result inputStatus_ = Result::InvalidInput;
  float systematic(unsigned branch, size_t t) const;
  float parity(unsigned branch, size_t t) const;
  void bcjr(unsigned branch);
};
constexpr size_t TransportDecoder::workspaceBytes() { return sizeof(TransportDecoder); }

// Supports the validated root SIB5/SIB8 container; rejects other root SIBs.
// Takes an already CRC-verified transport payload. No bit-pattern searching.
Result parseSib8(const uint8_t* payload, size_t bytes, Sib8Time& time);
Result toUtc(const Sib8Time& time, UtcTime& utc);

// Supply the *SI-window reference boundary*, not USB receipt/capture start.
// Fails closed on a reset, future reference or stale observation.
Result advanceToNow(const UtcTime& atBoundary, int64_t boundaryUptimeUs,
                    int64_t nowUptimeUs, uint64_t maxAgeUs, int64_t& unixUs);

// Decode the SI-RNTI convolutional control block (payload + inverted CRC16).
// Payload length <= 48 bits, output bits are bytes containing zero or one.
// Alternate CRC masks support PBCH. fastOnly selects a bounded circular search
// that requires a closed traceback and valid CRC; default is exact tail-biting.
Result decodeSiControl(const float* llr, size_t count, size_t payloadBits,
                       uint8_t* bits, size_t capacity, ServiceCallback service = nullptr,
                       uint16_t crcMask = 0xffff, bool fastOnly = false);
uint32_t crc24a(const uint8_t* data, size_t bytes);
}  // namespace esp32_lte_time
