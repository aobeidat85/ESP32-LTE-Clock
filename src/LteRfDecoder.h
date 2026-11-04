// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "LteTimeDecoder.h"

namespace esp32_lte_time {
struct Complex {
  float r, i;
  Complex operator+(Complex b) const { return {r+b.r,i+b.i}; }
  Complex operator-(Complex b) const { return {r-b.r,i-b.i}; }
  Complex operator*(Complex b) const { return {r*b.r-i*b.i,r*b.i+i*b.r}; }
  Complex operator*(float b) const { return {r*b,i*b}; }
  Complex operator/(float b) const { return {r/b,i/b}; }
  Complex conjugate() const { return {r,-i}; }
  float power() const { return r*r+i*i; }
};
struct Sync {
  bool found = false;
  uint16_t pci = 0, pssStart = 0;
  uint8_t subframe = 0;
  float pssScore = 0, sssScore = 0, carrierCorrectionHz = 0;
  double frameOffsetUs = 0;
};
struct Mib {
  bool valid = false;
  uint16_t sfn = 0, resourceBlocks = 0;
  uint8_t ports = 0;
  float phichResource = 0;
  bool extendedPhich = false;
};
struct SiSchedule {
  uint32_t cellId = 0;
  uint16_t periodFrames = 0, startMs = 0;
  uint8_t windowMs = 0, band = 0;
  bool hasSib8 = false;
};
Result parseSib1Schedule(const uint8_t* payload, size_t bytes, SiSchedule& schedule);
// Visit every millisecond of a received SI window, interleaving its frames.
// Returns -1 for an invalid window; repeats after windowMs attempts.
int siWindowOffset(unsigned windowMs, unsigned attempt);
bool fitFramePeriod(double firstUptimeUs, unsigned firstSfn, double lastUptimeUs,
                    unsigned lastSfn, double& periodUs);
Result sib8ReferenceUptime(const SiSchedule& schedule, unsigned capturedSfn, unsigned subframe,
                          uint64_t captureUptimeUs, int cpStartSample, double framePeriodUs,
                          int64_t& referenceUptimeUs);

// Initial live profile: LTE FDD normal CP, 50 RB, four antenna ports, Ng=1.
// All processing is local. Raw words are borrowed and never modified.
class RfDecoder {
 public:
  RfDecoder() = default;
  ~RfDecoder() { release(); }
  RfDecoder(const RfDecoder&) = delete;
  RfDecoder& operator=(const RfDecoder&) = delete;
  bool reserve();
  void release();
  ServiceCallback service = nullptr;
  bool (*cancelled)(void*) = nullptr;
  void* cancelContext = nullptr;
  uint8_t allocationFailure = 0; // 1=sample grid, 2=Turbo, 3=pilots, 4..7=scratch.
  float pssPeak = 0, sssPeak = 0;
  bool rawStable = true;
  Sync acquire(const uint32_t* raw, size_t count, double carrierOffsetHz, unsigned pci,
               float coarseHz = 0, bool sweepCfo = true);
  // Blind FDD discovery: all three PSS roots and all 168 SSS groups. No PCI hint.
  Sync discover(const uint32_t* raw, size_t count, double carrierOffsetHz,
                const uint16_t* excluded = nullptr, size_t excludedCount = 0);
  Mib readMib(const Sync& sync); // Uses the narrow samples left by acquire().
  Result readSystem(const uint32_t* raw, size_t count, double carrierOffsetHz,
                    unsigned pci, unsigned subframe, uint8_t* payload, size_t& bytes);
  // Diagnostics from the last public SI attempt, also its RF time reference.
  int timingStart = 0;
  float cpScore = 0, appliedCfoHz = 0;
  unsigned grants = 0, transportBits = 0;
  static constexpr size_t workspaceBytes();
  uintptr_t arenaAddress() const { return reinterpret_cast<uintptr_t>(sampleBanks_[0]); }
 private:
  // Logical shared sample arena. The first bank can live in 32-bit-only IRAM;
  // all access there is explicitly volatile uint32_t, never float or halfword.
  static constexpr unsigned StorageWords = 14*600*2;
  static constexpr unsigned IramWords = 10240;
  uint32_t* sampleBanks_[2] = {};
  Complex* fft_ = nullptr;
  Complex* ring_ = nullptr;
  Complex (*pilots_)[4][100] = nullptr;
  float* powers_ = nullptr;
  float* control_ = nullptr;
  TransportDecoder* turbo_ = nullptr;
  static constexpr unsigned PowerCapacity = 2048;
  size_t timeCount_ = 0, narrowCount_ = 0;
  unsigned pci_ = 0, subframe_ = 0;
  double carrierOffset_ = 0;
  bool stopping() const { return cancelled && cancelled(cancelContext); }
  uint32_t loadWord(size_t index) const;
  void storeWord(size_t index, uint32_t value);
  Complex loadComplex(size_t wordIndex) const;
  void storeComplex(size_t wordIndex, Complex value);
  Complex narrow(size_t index) const { return loadComplex(index*2); }
  Complex gridValue(unsigned symbol,unsigned sc) const { return loadComplex((symbol*600+sc)*2); }
  Complex timeSample(size_t index) const;
  bool resample(const uint32_t* raw, size_t count, double offset, bool narrow);
  bool grid(int start, float cfo);
  void estimate();
  Complex channel(unsigned port, unsigned sym, unsigned sc) const;
  void equalize(const Complex* data, const uint16_t* sc, const uint8_t* sym,
                unsigned count, unsigned globalOffset, Complex* out, float* power) const;
  unsigned controlRegion(unsigned cfi);
  Result sharedChannel(unsigned firstRb, unsigned rbs, unsigned cfi, unsigned rv,
                       unsigned tbs, uint8_t* payload);
};
constexpr size_t RfDecoder::workspaceBytes() {
  return sizeof(RfDecoder)+StorageWords*sizeof(uint32_t)+sizeof(Complex)*(2048+256+1600)
    +sizeof(float)*(PowerCapacity+16*72)+sizeof(TransportDecoder);
}
} // namespace esp32_lte_time
