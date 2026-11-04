# Third-party notices

## ESP32-Harmonic-RX and ESPARGOS / ESP-SDR

This standalone library was extracted from the LTE clock work in
https://github.com/aobeidat85/ESP32-Harmonic-RX. The raw RF backend is retained
under `src/detail/` with unrelated tone/weather capture removed.

The original-ESP32 RF register sequence, SRAM reservation and tuning approach
were adapted from https://github.com/ESPARGOS/esp-sdr at revision
`9b61e7465bee6da15209c5c09308043d08c46a1d`, principally
`main/targets/esp32/receiver.c` and `sram_guard.ld`. That upstream source is GPLv3.
This library is distributed as GPL-3.0-only; the full license is in `LICENSE`.

## Daniel Estevez / LTE Turbo codec

`src/LteTimeDecoder.cpp` adapts Turbo rate matching, terminated BCJR and iterative
decoding algorithms from `LTE/lte.py` in
https://github.com/daniestevez/jupyter_notebooks at revision
`6c923c4f22b684873a2f9932f9e78f05b53fc324`.
The unmodified MIT reference source and full MIT permission notice are retained
in `tests/reference/vendor/`. Reference code is used only by developer tests and
table generation; it is not an on-device Python dependency.

## Standards, schema and time tables

PSS/SSS, OFDM, CRS and control/data decoding follow 3GPP TS 36.211/212/213.
Root SIB5/SIB8 schema metadata follows TS 36.331, generated through pycrate 0.8.1.
QPP coefficients follow TS 36.212. Generated FIR coefficients use SciPy's default
Kaiser polyphase filter parameters; SciPy is a developer dependency only.

The saved IERS table in `extras/iers-leap-seconds.dat` was preserved with the
hardware experiment. Source: https://hpiers.obspm.fr/iers/bul/bulc/Leap_Second.dat
(recorded Bulletin C 72 context). It expires on 2027-06-28.

## Espressif and Arduino

The pinned PlatformIO build obtains Arduino-ESP32 2.0.17 and its ESP-IDF/PHY
components. Those components retain their own licenses and are not vendored here.
Private PHY calls are a compatibility constraint, not a supported Espressif
public radio API. Product names identify compatibility and do not imply endorsement.
