// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "detail/Esp32RawRx.h"
#include "LteRfDecoder.h"

namespace esp32_lte_time {
enum class ClockStatus : uint8_t { Idle, Searching, Acquiring, ReadingSchedule, ReadingTime,
                                  Synced, TimedOut, RadioError, UnsupportedCell, ConversionError, NoMemory };
struct ClockConfig {
  // Initial experimental profile, verified locally. Not a universal LTE scan.
  uint16_t tuningMHz = 2149;
  double carrierHz = 2150000000.0;
  uint16_t pcis[2] = {284,283};
  uint8_t gainIndex = 68;
  uint64_t maxObservationAgeUs = 120000000;
};
struct ClockEvidence {
  uint16_t pci = 0, sfn = 0;
  uint32_t cellId = 0, captures = 0, mibs = 0;
  uint64_t rawTicks = 0;
  int64_t referenceUptimeUs = 0, utcAtReferenceUs = 0;
  double framePeriodUs = 0;
  float pssScore = 0, sssScore = 0;
  int timingSample = 0;
  uint8_t radioError = 0;
  uint8_t allocationFailure = 0;
  bool rawStable = true;
  float carrierCorrectionHz = 0;
  uint8_t timePayload[MaxTransportBits/8] = {};
  uint8_t timePayloadBytes = 0;
};
using ClockProgress = void (*)(ClockStatus, const ClockEvidence&);

// Allocate globally/static: owns approximately 133 KB of reusable DSP storage.
// Original dual-core ESP32, Arduino loop/core1, Wi-Fi and Bluetooth inactive.
// No host clock, USB command, saved frame counter, or saved UTC input is used.
class Esp32LteClock {
 public:
  bool sync(uint32_t timeoutMs = 180000, const ClockConfig& config = ClockConfig(),
            ClockProgress progress = nullptr);
  ClockStatus status() const { return status_; }
  const ClockEvidence& evidence() const { return evidence_; }
 private:
  esp32_lte_raw::Receiver radio_;
  RfDecoder rf_;
  ClockConfig config_;
  ClockEvidence evidence_;
  ClockProgress progress_ = nullptr;
  ClockStatus status_ = ClockStatus::Idle;
  int64_t deadline_ = 0;
  double phase_ = 0, period_ = 10000.2459, carrier_ = 0;
  uint16_t pci_ = 0, sfn_ = 0;
  bool expired() const;
  void report(ClockStatus status);
  bool capture(const uint32_t*& words, uint64_t& stamp, uint64_t target = 0);
  bool scheduled(const uint32_t*& words, uint64_t& stamp, unsigned period,
                 unsigned remainder, unsigned subframe, unsigned& capturedSfn);
  bool acquire(const uint32_t* words, uint64_t stamp, Sync candidate);
  bool track();
};
} // namespace esp32_lte_time
