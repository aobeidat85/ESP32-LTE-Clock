# ESP32-LTE-Clock

Experimental UTC synchronization from public LTE broadcasts using an **original
ESP32 and its onboard antenna**. RF processing, cell acquisition, decoding and
clock setting run on the ESP32. No LTE modem, SIM, GPS module, Internet connection
or PC decoder is required at runtime.

**Tested board: an original dual-core ESP32 development board carrying an
ESP32-D0WD-V3, revision 3.1**, running at 240 MHz with a 40 MHz crystal and onboard
antenna. `esp32dev` is the PlatformIO build target. A specific development-board
manufacturer/model has not been established. ESP32-C3/S3/C5 and other variants
are not supported by this backend.

This is a narrow, measured experimental profile: Verizon LTE near 2150 MHz,
PCI 284/283, FDD normal CP, 50 resource blocks, four antenna ports and Ng=1.
It is **not a general LTE-band scanner**. Changing the configuration does not
establish compatibility with another carrier, band or board.

## Simple example

```cpp
#include <Esp32LteClock.h>

static esp32_lte_time::Esp32LteClock lteClock;

void setup() {
  if (lteClock.sync(180000)) {
    // UTC is set. Use time(), gettimeofday(), or gmtime_r().
  } else {
    // No verified time was obtained. Retry or handle an unset clock.
  }
}

void loop() {}
```

The [SetUtcOnBoot example](examples/SetUtcOnBoot/SetUtcOnBoot.ino) retries after
failure and prints UTC once synchronized. Serial output is optional diagnostics;
the receiver accepts no serial commands or host time. Search stops and the radio
and DSP allocations are released after success or timeout.

## Build and install

Use the pinned PlatformIO setup. The private PHY backend is qualified only with
**Arduino-ESP32 2.0.17**, supplied by `espressif32@6.12.0`, on core1 at 240 MHz.
The library adds its required SRAM linker guards through its PlatformIO build
hook. Generic Arduino IDE installation has not been qualified.

```text
git clone https://github.com/aobeidat85/ESP32-LTE-Clock.git
cd ESP32-LTE-Clock
pio run -d examples/SetUtcOnBoot
pio run -d examples/SetUtcOnBoot -t upload --upload-port COM10
pio device monitor --port COM10 --baud 115200
```

Replace `COM10` with your board's port. The example's local library dependency
points to this checkout. For another PlatformIO application:

```ini
[env:esp32]
platform = espressif32@6.12.0
board = esp32dev
framework = arduino
board_build.f_cpu = 240000000L
lib_deps = https://github.com/aobeidat85/ESP32-LTE-Clock.git#v0.1.0
build_flags = -O3
```

Keep Wi-Fi and Bluetooth inactive during synchronization. The receiver needs
exclusive ownership of the radio. It reserves a 64 KB capture aperture and uses
about 133 KB of reusable DSP workspace, including a 40 KB integer-access IRAM
bank. Insufficient memory or an unsupported configuration returns failure.

## Test findings

The integrated prototype passed a user-performed USB **power-off/reconnect test**:
it received fresh LTE time and set UTC after **137.384 seconds**, with no host
time or decoder input. A fresh-reset fast-build test took **190.989 seconds**,
including a timed-out attempt and retry. These are observations, not guaranteed
acquisition times. The standalone extraction retains **49 automated tests**.

See [test findings and hardware details](docs/TEST_FINDINGS.md) and the
[machine-readable power-cycle evidence](docs/evidence/physical-power-cycle-results.json).
Historical prototype evidence is clearly separated from package validation.

## Configuration and limits

`sync(timeoutMs, config, progress)` accepts a `ClockConfig` and an optional
progress callback. The default requests LO 2149 MHz, carrier 2150 MHz, PCI 284/283
and gain-table index 68. This index is not calibrated dB. A timeout is checked
between bounded DSP operations and can overrun while the current operation ends.

The implementation requires fresh agreeing MIBs, fresh SIB1 scheduling, SI-RNTI
CRC16 and transport CRC24 before accepting SIB8 time. It converts GPS-scale time
to UTC with the included IERS leap-second table and accounts for elapsed device
time at the SI-window reference boundary. A saved calendar or frame number is
never used to establish time.

Encoded time resolution is 10 ms for the tested synchronous field; **absolute
UTC accuracy has not been calibrated**. The leap table expires **2027-06-28**;
update it before that date. Expired or unrepresentable conversions fail. Root
SIB16, distributed VRBs, multiple code blocks, and transport payloads above
488 bits are outside this release.

## Developer checks

```text
python -m pip install -r requirements-dev.txt
python -m pytest tests -q
pio run -d examples/SetUtcOnBoot
```

Native tests build the same decoder with g++ on Linux or MSVC on Windows.
Reference Python code and RF fixtures are test-only; firmware has no Python
dependency and the simple example contains no historical RF replay data.

GPL-3.0-only. See [third-party notices](THIRD_PARTY_NOTICES.md).
