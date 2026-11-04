// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "detail/Esp32RawRx.h"
#include "LteRfDecoder.h"
#include "LteCellScan.h"

namespace esp32_lte_time {
enum class ClockStatus : uint8_t { Idle, Searching, Acquiring, ReadingSchedule, ReadingTime,
                                  Synced, TimedOut, RadioError, UnsupportedCell, ConversionError, NoMemory };
struct ClockConfig {
  // Default: scan reachable FDD ranges and discover PCI from PSS/SSS.
  bool autoScan = true;
  uint8_t capturesPerFrequency = 24;
  uint32_t cellTimeoutMs = 180000;
  uint8_t cellAttempts = 3;
  // Optional legacy/manual mode: set autoScan=false to use these hints.
  uint16_t tuningMHz = 0;
  double carrierHz = 0;
  uint16_t pcis[2] = {0,0};
  uint8_t gainIndex = 68;
  uint64_t maxObservationAgeUs = 120000000;
};
struct ClockEvidence {
  uint32_t scanVisits = 0;
  uint16_t carrier100kHz = 0, tuningMHz = 0;
  uint16_t discoveredCells = 0, rejectedCells = 0;
  uint16_t pci = 65535, sfn = 0; // No synchronization hypothesis yet.
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
  bool sync(uint32_t timeoutMs = 600000, const ClockConfig& config = ClockConfig(),
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
  int64_t cellDeadline_ = 0;
  uint32_t scanCursor_ = 0; // Continues across timeout/retry on this instance.
  uint16_t activeTuningMHz_ = 0;
  double activeCarrierHz_ = 0;
  double phase_ = 0, period_ = 10000.2459, carrier_ = 0;
  uint16_t pci_ = 0, sfn_ = 0;
  bool expired() const;
  void report(ClockStatus status);
  bool capture(const uint32_t*& words, uint64_t& stamp, uint64_t target = 0);
  bool scheduled(const uint32_t*& words, uint64_t& stamp, unsigned period,
                 unsigned remainder, unsigned subframe, unsigned& capturedSfn);
  bool acquire(const uint32_t* words, uint64_t stamp, Sync candidate);
  bool track();
  bool receiveTime(const uint32_t* words, uint64_t stamp, Sync found);
};
} // namespace esp32_lte_time
