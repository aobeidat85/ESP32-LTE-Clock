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

**Automatic frequency and PCI discovery is the default.** The device scans
reachable LTE FDD downlink ranges and derives the physical cell ID from received
PSS/SSS, with no configured carrier or PCI. It tries discovered cells for fresh
MIBs and time information, then continues scanning when a cell is unsuitable.

UTC decoding remains experimental: the qualified time-bearing profile is FDD
normal CP, 50 resource blocks, four antenna ports, Ng=1 and SIB8 GPS-scale time.
Finding another LTE cell does not establish that its time message is supported.

## Simple example

```cpp
#include <Esp32LteClock.h>

static esp32_lte_time::Esp32LteClock lteClock;

void setup() {
  if (lteClock.sync()) {
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
lib_deps = https://github.com/aobeidat85/ESP32-LTE-Clock.git#v0.2.0
build_flags = -O3
```

Keep Wi-Fi and Bluetooth inactive during synchronization. The receiver needs
exclusive ownership of the radio. It reserves a 64 KB capture aperture and uses
about 141 KB of reusable DSP workspace, including a 40 KB integer-access IRAM
bank. Insufficient memory or an unsupported configuration returns failure.

## Test findings

The integrated prototype passed a user-performed USB **power-off/reconnect test**:
it received fresh LTE time and set UTC after **137.384 seconds**, with no host
time or decoder input. A fresh-reset fast-build test took **190.989 seconds**,
including a timed-out attempt and retry. These are observations, not guaranteed
acquisition times. The library now passes **67 automated tests**.

See [test findings and hardware details](docs/TEST_FINDINGS.md) and the
[machine-readable power-cycle evidence](docs/evidence/physical-power-cycle-results.json).
Historical prototype evidence is clearly separated from package validation.
The extracted standalone example was also flashed to the same board and
successfully set UTC from fresh live LTE after an upload reset and one retry.
The **final automatic-scan firmware set UTC after 70.000 seconds from boot**:
it scanned from 2155 to 2150 MHz, derived PCI 284 and read fresh time without
frequency/PCI hints. An earlier automatic test took 151.740 seconds, including
fallback from PCI 283 to PCI 284. The findings retain unsuccessful observations
as well. These tests used RTS reset; the physical power-cut result above is
historical prototype evidence.

## Configuration and limits

`sync(timeoutMs, config, progress)` accepts a `ClockConfig` and optional progress
callback. `sync()` uses a ten-minute budget and automatic scanning. A failed
attempt leaves the scan cursor ready to resume at the next frequency on the next
call, as the example does. UTC can only be obtained when a receivable cell
broadcasts a supported time message.

The scan searches the following unions of FDD downlink ranges, in MHz:
**2110–2199.9, 1930–1994.9, 1842–1879.9, 2350–2359.9, 2620–2689.9**.
These cover the reachable portions of bands 1/2/3/4/7/9/10/23/25/30 and overlapping
extensions. Lower bands and TDD are outside this backend. The channel ranges
follow [ETSI TS 136 101, section 5.7.3](https://www.etsi.org/deliver/etsi_ts/136100_136199/136101/17.15.00_60/ts_136101v171500p.pdf).

A first pass probes 5 MHz centers from each range's middle outward, followed by
every remaining 100 kHz point. A complete pass has **2,730 distinct frequencies**.
The coarse pass prioritizes convenient centers; it does not guarantee that a
cell is on one. Every point remains in the full scan. There is no learned bench
frequency or PCI in the default configuration. Blind PSS correlations use FFTs;
all 504 PCI values are eligible through SSS correlation.

Defaults take 24 snapshots per frequency and allow up to 180 seconds for an
individual attempt, with up to three acquisition attempts per discovered PCI
before moving on. A verified unsupported profile or time schedule is skipped
immediately. A cell that advertises time but exhausts its scheduled time attempts
yields to another PCI on that carrier instead of consuming every retry budget.
Time acquisition searches offsets throughout the received SIB1 SI window,
interleaving its radio frames and retries at the window start to tolerate fading;
it does not require transmission at the first subframe. The per-cell deadline
still applies during this search.
Rejected PCIs are skipped for the remainder of that dwell, so another cell can
be considered. Multiple scans or a long search may be needed;
a full raster scan can take hours. The timeout is checked between bounded DSP
operations and can overrun while the current operation ends. Gain-table index
68 is configurable and is not calibrated dB.

For a constrained application, `ClockConfig` exposes `capturesPerFrequency`,
`cellTimeoutMs`, `cellAttempts`, and `gainIndex`. Manual mode is still available: set
`autoScan=false` and explicitly supply `tuningMHz`, `carrierHz`, and `pcis`.
Those manual hint fields default to zero and are ignored during automatic scan.

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
For optional hardware observation, install `pyserial` and run
`python tools/observe_boot.py --reset --log build/boot.jsonl`. RTS resets the
board; the observer sends zero serial data bytes and never supplies time.

GPL-3.0-only. See [third-party notices](THIRD_PARTY_NOTICES.md).
