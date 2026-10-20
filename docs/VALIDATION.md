# P4 UVC implementation validation — 2026-10-19

## Completed

- Full ESP32-P4 firmware and bootloader build using ESP-IDF v5.5.1
  (fcae32885b0296b32044cb99ecbdc50d98dddb83), GCC 14.2.0
  esp-14.2.0_20241119. Partition size check passed.
- Configuration verified: high-speed USB OTG, one MJPEG format, 480×320,
  4 fps, multiple resolutions disabled, USB Serial/JTAG console, PSRAM at
  80 MHz, supported silicon revisions 0.1–1.99.
- Host regression test passed with AddressSanitizer and UBSan: entirely
  invalid frame rejection; isolated infinity/NaN repair; unrepairable corner
  cluster rejection; excessive invalid pixel rejection; true extrema;
  minimum display span; fixed modes; renderer buffer guards.
- Generated and visually inspected a 480×320 gradient preview including
  top-bar controls, measured extrema, age, repaired count and colour scale.
- Verified BGR888 layout against ESP-IDF JPEG documentation and verified UVC
  get/return buffer ownership against the pinned driver's implementation.

## Local build environment details

The container's process namespace prevents the newest component manager from
querying its own process with psutil. Component manager 2.2.0 (within IDF's
required version range), CMake 3.30.9 and Ninja 1.13.2 were used instead.
LeakSanitizer also cannot enumerate /proc here; leak detection was disabled
for the host test, leaving address and undefined-behaviour checks enabled.

Fetching the unused ESP-IDF MQTT submodule failed. That incomplete component
was moved outside the SDK component search directory locally; it is not a
firmware dependency. The used SDK component sources were not patched. SDK
configuration reports unrelated incomplete/out-of-date unused submodules;
compilation of this application completed without compiler warnings. The
repository does not contain this local SDK adjustment. A normal complete
ESP-IDF v5.5.1 checkout should use the README's standard build procedure.

## Not established by these checks

No physical board was attached, flashed or measured. USB enumeration, sustained
host streaming, JPEG colours on hardware, LCD orientation, touch calibration,
sensor reconnection and actual fresh-frame cadence require the hardware
acceptance checks in README.md. Do not treat successful compilation as proof
of those behaviours. This build does not support P4X/revision 3.x.
