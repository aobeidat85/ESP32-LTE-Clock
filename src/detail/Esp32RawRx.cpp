// SPDX-License-Identifier: GPL-3.0-only
// Capture registers and reserved aperture adapted from ESPARGOS/esp-sdr,
// commit 9b61e7465bee6da15209c5c09308043d08c46a1d. See THIRD_PARTY_NOTICES.md.
#include "Esp32RawRx.h"
#include <math.h>
#include "esp_arduino_version.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "heap_memory_layout.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "soc/rtc.h"
#include "soc/soc.h"

#if ESP_ARDUINO_VERSION_MAJOR != 2 || ESP_ARDUINO_VERSION_MINOR != 0 || ESP_ARDUINO_VERSION_PATCH != 17
#error "This private-PHY backend is qualified only with Arduino-ESP32 2.0.17. Use the pinned PlatformIO example."
#endif

SOC_RESERVE_MEMORY_REGION(0x3ffe8000, 0x3fff8000, harmonic_rx_dump);

extern "C" {
void set_chanfreq(unsigned, unsigned);
void rom_set_rf_freq_offset(unsigned, unsigned, int);
unsigned ram_chip_i2c_readReg(unsigned, unsigned, unsigned);
void ram_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
void rom_pbus_workmode(void);
void rom_pbus_xpd_rx_on(int);
void rom_pbus_xpd_tx_off(void);
void rom_set_rxclk_en(int);
extern char _iram_end, _data_end, _bss_end;
}

namespace esp32_lte_raw {
namespace {
constexpr uint32_t DumpCtrl = 0x60033d90u;
constexpr uint32_t DumpStatus = 0x60033d94u;
constexpr uint32_t DumpBytes = 0x60033dc4u;
constexpr uint32_t RxGain = 0x3ff5c02cu;
constexpr uint32_t Sentinel = 0xa55aa55au;
constexpr uintptr_t ApertureStart = 0x3ffe8000u;
constexpr uintptr_t ApertureEnd = 0x3fff8000u;
volatile uint32_t *const samples = reinterpret_cast<volatile uint32_t *>(ApertureStart);
portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;
Receiver *radioOwner = nullptr;
inline uint32_t cycles() { uint32_t value; asm volatile("rsr.ccount %0" : "=a"(value)); return value; }

}  // namespace

bool Receiver::begin(const Config &config) {
  if (ready_ || radioOwner) return fail(Error::RadioInUse);
  if (config.tuningMHz < MinimumTuningMHz || config.tuningMHz > 2800 || config.gainIndex > 81) return fail(Error::InvalidConfig);
  if (getCpuFrequencyMhz() != 240) return fail(Error::UnsupportedCpu);
  if (xPortGetCoreID() != 1) return fail(Error::UnsupportedCore);
  if (reinterpret_cast<uintptr_t>(&_iram_end) > 0x400a8000u ||
      reinterpret_cast<uintptr_t>(&_data_end) > ApertureStart ||
      reinterpret_cast<uintptr_t>(&_bss_end) > ApertureStart) return fail(Error::UnsafeMemoryLayout);
  wifi_mode_t mode;
  if (esp_wifi_get_mode(&mode) == ESP_OK) return fail(Error::RadioInUse);
  esp_err_t event = esp_event_loop_create_default();
  if (event != ESP_OK && event != ESP_ERR_INVALID_STATE) return fail(Error::WifiInitializationFailed);
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  if (config.minimalWifiBuffers) {
    init.static_rx_buf_num = 2;
    init.dynamic_rx_buf_num = 4;
    init.tx_buf_type = 1;
    init.static_tx_buf_num = 0;
    init.dynamic_tx_buf_num = 4;
    init.cache_tx_buf_num = 0;
    init.ampdu_rx_enable = 0;
    init.ampdu_tx_enable = 0;
    init.amsdu_tx_enable = 0;
    init.mgmt_sbuf_num = 6;
  }
  if (esp_wifi_init(&init) != ESP_OK) return fail(Error::WifiInitializationFailed);
  if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
      esp_wifi_set_mode(WIFI_MODE_NULL) != ESP_OK || esp_wifi_start() != ESP_OK ||
      esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK || esp_wifi_set_promiscuous(true) != ESP_OK ||
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    esp_wifi_stop(); esp_wifi_deinit();
    return fail(Error::WifiInitializationFailed);
  }
  config_ = config;
  prepare();
  radioOwner = this; ready_ = true; error_ = Error::None;
  return true;
}

void Receiver::end() {
  if (!ready_ || radioOwner != this) return;
  REG_WRITE(DumpCtrl, 0);
  esp_wifi_set_promiscuous(false); esp_wifi_stop(); esp_wifi_deinit();
  ready_ = false; radioOwner = nullptr;
}

void Receiver::prepare() {
  REG_WRITE(DumpCtrl, 0);
  // Pin the normal LO divider before selecting the requested receive profile.
  // A previous alternate-LO experiment must not leak into this profile.
  const unsigned lo = ram_chip_i2c_readReg(0x65, 4, 0);
  ram_chip_i2c_writeReg(0x65, 4, 0, lo & ~0x10u);
  set_chanfreq(2412, 0);
  const unsigned cap = ram_chip_i2c_readReg(0x62, 1, 0);
  ram_chip_i2c_writeReg(0x62, 1, 0, cap & ~0x80u);
  const unsigned xtal = rtc_clk_xtal_freq_get();
  // Upstream original-ESP32 alternate LO divides PLL frequency by 6/5.
  // Normal mode is retained for vendor calibration and normal-mode profiles.
  const bool alternate = 1 &&
                         config_.tuningMHz >= 1842 && config_.tuningMHz < 2210;
  const unsigned pllKHz = config_.tuningMHz * (alternate ? 1200u : 1000u);
  const int offset = ((pllKHz % 1000u) * 1024u + 500u) / 1000u;
  rom_set_rf_freq_offset(xtal == 26 ? 1 : xtal == 24 ? 2 : 0, pllKHz / 1000u, offset);
  rom_pbus_workmode(); rom_pbus_xpd_tx_off(); rom_pbus_xpd_rx_on(1); rom_set_rxclk_en(1);
  const unsigned activeLo = ram_chip_i2c_readReg(0x65, 4, 0);
  ram_chip_i2c_writeReg(0x65, 4, 0, (activeLo & ~0x10u) | (alternate ? 0x10u : 0u));
  const uint32_t gain = REG_READ(RxGain);
  REG_WRITE(RxGain, (gain & 0x7fffff) | (config_.gainIndex << 24) | BIT(23));
  delay(3);
}

CaptureResult Receiver::captureRaw(uint32_t *output, size_t count, uint64_t *startedUptimeUs,
                                   uint64_t targetUptimeUs) {
  return captureRawImpl(output, count, startedUptimeUs, targetUptimeUs, false);
}

CaptureResult Receiver::captureRawView(const uint32_t*& output, size_t count,
                                      uint64_t* startedUptimeUs, uint64_t targetUptimeUs) {
  output = nullptr;
  CaptureResult result = captureRawImpl(nullptr, count, startedUptimeUs, targetUptimeUs, true);
  if (result.error == Error::None) output = reinterpret_cast<const uint32_t*>(ApertureStart);
  return result;
}

CaptureResult Receiver::captureRawImpl(uint32_t *output, size_t count, uint64_t *startedUptimeUs,
                                      uint64_t targetUptimeUs, bool borrowed) {
  CaptureResult result;
  if (!ready_) { result.error = Error::NotReady; return result; }
  if (getCpuFrequencyMhz() != 240) { result.error = Error::UnsupportedCpu; return result; }
  if (xPortGetCoreID() != 1) { result.error = Error::UnsupportedCore; return result; }
  const uintptr_t start = reinterpret_cast<uintptr_t>(output);
  const uintptr_t end = start + count * sizeof(uint32_t);
  if (count < 256 || count > MaxRawSamples || (!borrowed &&
      (!output || (start & 3u) || end < start || (start < ApertureEnd && end > ApertureStart)))) {
    result.error = Error::InvalidBuffer; return result;
  }
  prepare();
  const uint64_t scheduleNow = esp_timer_get_time();
  if (targetUptimeUs && (targetUptimeUs < scheduleNow + 300 ||
                        targetUptimeUs > scheduleNow + 3000000)) {
    result.error = Error::InvalidBuffer; return result;
  }
  for (unsigned k = 0; k < 16384; ++k) samples[k] = Sentinel;
  const uint32_t owner = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
  const uint32_t byteSelect = REG_READ(DumpBytes);
  unsigned savedFilter[2];
  for (unsigned k = 0; k < 2; ++k) {
    savedFilter[k] = ram_chip_i2c_readReg(0x67, 1, 1 + k);
    ram_chip_i2c_writeReg(0x67, 1, 1 + k, (savedFilter[k] & ~127u) | 127u);
  }
  delayMicroseconds(10);
  if (targetUptimeUs) {
    while (static_cast<int64_t>(targetUptimeUs - esp_timer_get_time()) > 3000) delay(1);
    while (static_cast<int64_t>(targetUptimeUs - esp_timer_get_time()) > 80) {}
  }
  portENTER_CRITICAL(&captureMux);
  DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,
      (owner & ~DPORT_MAC_DUMP_MODE_M) | (3 << DPORT_MAC_DUMP_MODE_S));
  REG_WRITE(DumpBytes, 0x03020100);
  REG_WRITE(DumpCtrl, count | BIT(16) | BIT(31)); // Original ESP32 16 MS/s clock.
  if (targetUptimeUs) {
    while (static_cast<uint64_t>(esp_timer_get_time()) < targetUptimeUs) {}
  }
  const uint64_t triggerUptimeUs = esp_timer_get_time();
  const uint32_t began = cycles();
  REG_SET_BIT(DumpCtrl, BIT(19)); REG_CLR_BIT(DumpCtrl, BIT(19));
  while (!(REG_READ(DumpCtrl) & BIT(18)) && static_cast<uint32_t>(cycles() - began) < 480000u) {}
  const uint32_t done = REG_READ(DumpCtrl), status = REG_READ(DumpStatus);
  result.elapsedUs = (cycles() - began) / 240;
  REG_WRITE(DumpCtrl, 0);
  // Match upstream's post-stop settling delay before returning the bank to the CPU.
  delayMicroseconds(10);
  DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner);
  REG_WRITE(DumpBytes, byteSelect);
  portEXIT_CRITICAL(&captureMux);
  for (unsigned k = 0; k < 2; ++k) ram_chip_i2c_writeReg(0x67, 1, 1 + k, savedFilter[k]);
  if (!(done & BIT(18)) || (status & 0x7fff) != count) {
    result.error = Error::CaptureFailed;
  } else {
    for (unsigned k = 0; k < 16384; ++k) {
      if ((k < count && samples[k] == Sentinel) || (k >= count && samples[k] != Sentinel)) {
        result.error = Error::MemoryGuard; break;
      }
    }
    if (result.error == Error::None) {
      if (!borrowed) for (size_t k = 0; k < count; ++k) output[k] = samples[k];
      if (startedUptimeUs) *startedUptimeUs = triggerUptimeUs;
      result.count = count;
    }
  }
  error_ = result.error;
  return result;
}

const char *Receiver::errorMessage(Error error) {
  switch (error) {
    case Error::None: return "OK";
    case Error::NotReady: return "Receiver is not initialized";
    case Error::InvalidConfig: return "Invalid tuning or gain";
    case Error::UnsupportedCpu: return "CPU must run at 240 MHz";
    case Error::UnsupportedCore: return "Call the receiver from Arduino loop on core1";
    case Error::UnsafeMemoryLayout: return "RF capture aperture overlaps application memory";
    case Error::RadioInUse: return "The library requires exclusive ownership of the Wi-Fi radio";
    case Error::WifiInitializationFailed: return "Wi-Fi PHY initialization failed";
    case Error::InvalidBuffer: return "Invalid capture buffer or duration";
    case Error::CaptureFailed: return "No valid RF snapshots captured";
    case Error::MemoryGuard: return "RF capture aperture guard changed";
  }
  return "Unknown receiver error";
}
}  // namespace esp32_lte_raw

