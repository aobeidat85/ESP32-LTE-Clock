// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <Arduino.h>
#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "ESP32-LTE-Clock requires the original dual-core ESP32."
#endif
namespace esp32_lte_raw {
constexpr uint16_t MinimumTuningMHz = 1842;
struct Config {
  uint16_t tuningMHz = 2149;
  uint8_t gainIndex = 68;
  bool minimalWifiBuffers = true;
};
enum class Error : uint8_t {
  None, NotReady, InvalidConfig, UnsupportedCpu, UnsupportedCore,
  UnsafeMemoryLayout, RadioInUse, WifiInitializationFailed,
  InvalidBuffer, CaptureFailed, MemoryGuard
};
struct CaptureResult {
  size_t count = 0;
  uint32_t elapsedUs = 0;
  Error error = Error::None;
};
class Receiver {
 public:
  Receiver() = default;
  ~Receiver() { end(); }
  Receiver(const Receiver&) = delete;
  Receiver& operator=(const Receiver&) = delete;
  static constexpr size_t MaxRawSamples = 16380;
  bool begin(const Config& config = Config());
  bool tune(uint16_t tuningMHz);
  void end();
  Error lastError() const { return error_; }
  CaptureResult captureRaw(uint32_t* output,size_t count,uint64_t* startedUptimeUs=nullptr,uint64_t targetUptimeUs=0);
  // Read-only lease, on core1; invalidated by the next capture/end. Never share between tasks.
  CaptureResult captureRawView(const uint32_t*& output,size_t count,uint64_t* startedUptimeUs=nullptr,uint64_t targetUptimeUs=0);
  static const char* errorMessage(Error error);
 private:
  Config config_;
  bool ready_ = false;
  Error error_ = Error::None;
  void prepare();
  CaptureResult captureRawImpl(uint32_t* output,size_t count,uint64_t* startedUptimeUs,uint64_t targetUptimeUs,bool borrowed);
  bool fail(Error error) { error_=error; return false; }
};
} // namespace esp32_lte_raw
