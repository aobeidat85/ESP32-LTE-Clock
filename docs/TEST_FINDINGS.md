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

## v0.2 automatic discovery validation

Default configuration now scans frequency and derives PCI from PSS/SSS without
carrier/PCI hints. The full deterministic plan contains 2,730 distinct 100 kHz
points over five reachable FDD downlink ranges. It tries 5 MHz centers first,
then all other raster points. Each frequency gets 24 snapshots; a candidate gets
up to 180 seconds per attempt for fresh MIBs, SIB1 scheduling and SIB8 time.
Fading/CRC failures permit up to three acquisition attempts per discovered PCI;
a verified unsupported profile or schedule is excluded immediately. After a cell
advertises time but exhausts its time-reading attempt, another PCI on the same
carrier gets a fresh dwell. A timeout/retry
continues the scan cursor. These limits bound attempts, not total acquisition
time; a complete raster search can take hours.

**67 automated tests passed** locally, including the existing end-to-end recorded
RF/time tests and new blind-discovery tests. Independent NumPy/SciPy vectors
cover all three PSS roots, both FDD synchronization positions, PCI values 0, 1,
2, 57, 283 and 503, frequency offsets and noise. Tests also check recorded MIB
discovery with no PCI, exclusions, cancellation, noise/flat rejection and full
scan-raster coverage without duplicates. SI-window traversal tests cover every
millisecond of all seven permitted window lengths, interleave radio frames,
verify repeat coverage and reject invalid windows. The final decoder searches
the announced window rather than assuming SIB8 is transmitted at its start.

**Automatic discovery obtained live UTC after 151.740 seconds from boot** on the
stated ESP32. It began at 2155 MHz, moved to 2150 MHz without a carrier hint,
derived PCI 283 and read its time schedule, then moved to another PCI after the
time attempt failed. It independently derived PCI 284, obtained agreeing MIBs
and fresh SIB8, and set UTC. Successful evidence was cell 45012513, 96 captures,
four MIBs and counter **147780007492**. An independent counter/table conversion
gives reference UTC **2026-11-04 04:00:56.920Z**; the board printed advancing UTC
at `04:00:57Z`, `04:00:58Z`, and `04:00:59Z`. No serial data bytes or host time
were supplied.

See [automatic UTC evidence](evidence/auto-scan-utc-results.json). This successful
hardware revision is commit `2b2150b`, preceding the full-window traversal
extension; the extension has its own tests. The binary hash was not retained
before that subsequent build, and is not reconstructed or inferred. This was
an RTS-reset test, not a new physical power-off test.

A separate fresh boot missed the time-bearing carrier during its snapshot dwell
and continued to other cells. That exploratory observation was stopped before
a complete scan; it did not obtain UTC. The
[retained trace](evidence/auto-scan-missed-dwell-results.json) demonstrates that
successful timing is not guaranteed for every boot.

An initial live scan on the stated board found an unconfigured **2140 MHz,
PCI 242** cell, decoded agreeing MIBs and SIB1, then rejected its schedule because
it did not advertise the supported SIB8 time message. The scan continued. A
later shorter-budget run found **2150 MHz, PCI 284** and its time schedule but
exhausted the initial 60-second per-cell budget before UTC. That observation led
to the release's 180-second budget. Synchronization hypotheses are not treated
as verified cells or time; MIB and transport CRC checks remain mandatory.
The [initial discovery trace](evidence/auto-scan-discovery-results.json) and
[short-budget trace](evidence/auto-scan-short-budget-results.json) retain both
observations. These are software-reset tests, not physical power-off tests.

Both the Linux native-test and ESP32 firmware-build jobs passed on GitHub
([v0.2 implementation validation](https://github.com/aobeidat85/ESP32-LTE-Clock/actions/runs/41769112164)).
The release example uses 730,973 bytes of flash and 43,284 bytes of static RAM;
dynamic workspace is approximately 141 KB, including the 40 KB IRAM arena.
All three capture-SRAM overlap assertions remain present in the link map.

A subsequent [full-window trial](evidence/auto-scan-window-trial-results.json)
obtained MIBs and SIB1 on PCI 284 but exhausted its time-reading budget without
UTC, then correctly tried PCI 283. That exploratory observation was stopped
during further acquisition. The final traversal interleaves retries at the
announced window start with other offsets, preserving repeated opportunities
there under fading while still searching the rest of the window. Automated
tests verify both complete offset coverage and these start-window retries.

Scanning does not extend the SIB8/time transport profile: 50 RB, four antenna
ports, Ng=1, normal-CP FDD remain the qualified time-decoder parameters. Discovery
at other frequencies is not evidence of calibrated RF sensitivity, complete
regional coverage or supported UTC messages on every carrier.

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

## v0.1 standalone package validation

The extracted **SetUtcOnBoot example also synchronized from live RF on the same
board** after an upload reset. Its first attempt timed out; the automatic retry
received PCI 284, cell 45012513, two valid MIBs across 69 captures and fresh SIB8
counter **147779827780**, then printed advancing UTC at
**2026-11-04 03:31:03Z**, `03:31:04Z`, and `03:31:05Z`.
The observer sent zero serial data bytes. Success appeared 269.378 seconds after
the observer started; observation began after the upload reset, so this is not a
boot-relative latency or a second physical power-cycle test.
See [standalone-library live evidence](evidence/standalone-library-live-results.json),
including the tested firmware SHA-256 and source commit.

The extracted package passed **49 tests** locally on Windows using MSVC and
Python (49 passed in 66.69 seconds). Its standalone example built successfully
with the pinned PlatformIO configuration: 726,573 bytes of flash and 43,228 bytes
of static RAM, excluding dynamic workspace and the reserved capture aperture.
The link map confirms all three SRAM overlap assertions were applied.
An independent application also installed the public GitHub library through
`lib_deps` and built successfully, without referencing the original experiment
checkout or adding its own linker script.
GitHub Actions also passed the Linux native-test and ESP32 firmware-build jobs
([validation run](https://github.com/aobeidat85/ESP32-LTE-Clock/actions/runs/41767439026)).

The tests cover:

- Recorded raw I/Q to PSS/SSS, CRC-valid MIB, SIB1 schedule and SIB8 UTC.
- Independent Turbo encoder/rate-matching vectors for all redundancy versions,
  with noise and erasures.
- Exact and fast closed-path control readers with CRC verification.
- Missing/constant/noise/nonfinite input rejection and malformed/truncated RRC.
- Leap-second transitions and expiry, reset/stale monotonic references, SFN wrap
  and the SI-window reference boundary.

On the recorded coarse-frequency acquisition vector, MSVC and GCC can choose
adjacent correlation-peak samples. The test permits one 1.92 MS/s sample of
timing difference while requiring identical CRC-valid MIB fields.

Firmware builds use the pinned board/core configuration and SRAM linker guards.
The [simple example](../examples/SetUtcOnBoot/SetUtcOnBoot.ino) contains no replay
fixture. Python, NumPy and test reference modules are not firmware dependencies.

This release qualifies the stated experimental profile. Broader carriers/bands,
other ESP32 variants, calibrated accuracy and application-load margins require
additional testing.
