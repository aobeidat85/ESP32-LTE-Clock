# Test findings

## Tested ESP board

| Item | Verified configuration |
|---|---|
| Processor | **ESP32-D0WD-V3, revision 3.1** |
| Board family | Original dual-core ESP32 development board |
| Development-board brand/model | Not independently identified |
| CPU / crystal | 240 MHz / 40 MHz |
| Receive antenna | Onboard antenna only |
| Build target | PlatformIO `esp32dev` |
| Arduino core | Arduino-ESP32 2.0.17 |
| Platform | `espressif32@6.12.0` |
| Qualified LTE profile | Verizon, approximately 2150 MHz, PCI 284/283, 50 RB, four ports, normal-CP FDD, Ng=1 |

Do not interpret the `esp32dev` target as identification of a particular board
manufacturer, or this result as support for ESP32-C3, S3, C5, or every ESP32 module.

## Hardware findings from the integrated prototype

The LTE clock was first implemented in
[ESP32-Harmonic-RX](https://github.com/aobeidat85/ESP32-Harmonic-RX), then extracted
into this independent library. The saved hardware evidence below refers to that
tested integrated prototype. Its firmware/source hashes are historical evidence,
not hashes of this repackaged repository. Device MAC identifiers are omitted
from the public evidence copies.

| Test | Result | Time from boot | Notes |
|---|---|---:|---|
| Earlier exact-reader fresh reset | UTC set | 285.674 s | One timeout and retry |
| Fast-reader fresh reset | UTC set | 190.989 s | One timeout; successful retry took about 5.5 s |
| **Physical USB power cut and reconnect** | **UTC set** | **137.384 s** | First attempt; no software reset or host data sent |

In the physical power-cycle test the ESP32 acquired cell **45012513**, PCI **284**,
received four valid MIBs across 90 capture windows and decoded SIB8 counter
**147779714948**. The GPS-scale value was converted on-device to reference UTC
**2026-11-04 03:12:11.480Z**, then advanced using the same boot's monotonic clock
before setting the system clock. Independent parsing after the test reproduced
the counter and UTC from the printed payload exactly.

The board then printed advancing UTC with the radio released. Heap after radio
and workspace release was **233,028 bytes**. USB provided power and diagnostic
output; it supplied no clock, decoding result, or serial data command.

- [Physical power-cycle evidence](evidence/physical-power-cycle-results.json)
- [Fast-reader reset evidence](evidence/live-boot-results.json)
- [Earlier baseline evidence](evidence/live-boot-baseline-results.json)

Absolute UTC accuracy was not measured. Host receipt comparisons are coarse
sanity checks, not a calibrated time-transfer measurement. Reception quality,
placement and carrier broadcasts affect whether and how quickly synchronization
succeeds. Failed attempts are retained in the findings rather than hidden.

## Standalone package validation

The extracted package passed **49 tests** locally on Windows using MSVC and
Python (49 passed in 66.69 seconds). Its standalone example built successfully
with the pinned PlatformIO configuration: 726,573 bytes of flash and 43,228 bytes
of static RAM, excluding dynamic workspace and the reserved capture aperture.
The link map confirms all three SRAM overlap assertions were applied.

The tests cover:

- Recorded raw I/Q to PSS/SSS, CRC-valid MIB, SIB1 schedule and SIB8 UTC.
- Independent Turbo encoder/rate-matching vectors for all redundancy versions,
  with noise and erasures.
- Exact and fast closed-path control readers with CRC verification.
- Missing/constant/noise/nonfinite input rejection and malformed/truncated RRC.
- Leap-second transitions and expiry, reset/stale monotonic references, SFN wrap
  and the SI-window reference boundary.

Firmware builds use the pinned board/core configuration and SRAM linker guards.
The [simple example](../examples/SetUtcOnBoot/SetUtcOnBoot.ino) contains no replay
fixture. Python, NumPy and test reference modules are not firmware dependencies.

This release qualifies the stated experimental profile. Broader carriers/bands,
other ESP32 variants, calibrated accuracy and application-load margins require
additional testing.
