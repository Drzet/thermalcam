# Copilot Instructions for `thermalcam`

## Overview
- Arduino-based ESP32-S3 sketch that reads an MLX90640 thermal sensor, renders to an ILI9488 480x320 display with XPT2046 touch, and serves MJPEG streaming via web UI.
- **Dual-core architecture:** Core 1 captures/renders/displays thermal frames; Core 0 handles HTTP server and JPEG encoding with double-buffered ping-pong handoff for parallelism.

## Key Files
- `sketch_dec6a.ino`: main app (setup/loop), rendering pipeline, UI drawing, touch handling, WiFi bootstrapping, MLX init/config, dual frame buffer allocation (PSRAM preferred), core 0 task spawn.
- `ThermalModes.{h,cpp}`: mode definitions (Auto/Body/Home), button debounce on `BUTTON_PIN_A`, helpers for mode names and temperature ranges.
- `wifiman.{h,cpp}`: toggles between STA preset networks and AP; credentials are hardcoded placeholders—do not commit real secrets; `wifiManToggleMode()` switches modes and falls back to AP on STA failure.
- `webs.{h,cpp}`: HTTP server exposing `/` (HTML UI), `/stream` (MJPEG multipart stream), `/snapshot` (single JPEG), `/api/status` (JSON with mode/min/max), `/api/mode?next=1` (advance mode). Includes RGB565→RGB888 encoder wrapper (placeholder for HW JPEG encoder integration).

## Runtime Patterns & Constraints
- **Double-buffering:** Two RGB565 frame buffers (`frameBufferA`/`frameBufferB`) with semaphore handoff (`frameReadySem`/`frameFreeSem`). Core 1 renders to one buffer while core 0 encodes/streams from the other—no mutex contention, maximum parallelism.
- **Core 1 (loop):** Capture MLX frame → sanitize NaNs → render RGB565 (384x288 bilinear resample from 32x24) → swap buffer → signal core 0 → draw to display → touch handling. Runs at ~8 Hz MLX refresh rate.
- **Core 0 (webServerTask):** Wait for new frame → encode JPEG → serve `/stream` (multipart boundary) or `/snapshot` → release buffer → repeat. Chunked sends with `delay(0)` yields.
- Rendering assumptions: MLX frame is 32x24 (`mlxFrame`), resampled bilinearly to 384x288 using precomputed `xMap/yMap`; palette is 256-entry gradient (`tempPalette`). Keep dimensions consistent if changing.
- Temperature range selection uses `getTemperatureRange()`—Auto mode uses frame min/max (enforced ≥0.5°C span), fixed modes use configured bounds.
- UI: top bar buttons (Save/WiFi/Mode) drawn once; touch logic maps raw 0–4095 to screen coords (calibration TODO). Mode changes update label on next draw.
- WiFi button and startup both call `wifiManToggleMode()`; after a successful connection, `websBegin(VIEW_W, VIEW_H)` must be called once to start HTTP server, then core 0 task spawned.
- MLX init: starts I2C at 400kHz, then bumps to 1.5MHz after configuration; sensor mode `MLX90640_INTERLEAVED`, resolution 18-bit, 8 Hz refresh.
- Display SPI set to 40 MHz; backlight `TFT_BL` set HIGH in setup. Touch uses `HSPI` separate from display SPI.
- JPEG encoding uses RGB565→RGB888 conversion + placeholder encoder in `webs.cpp`; integrate ESP32 HW encoder (`fmt2jpg` from esp_jpg_encode or TJpg library) for production performance.
- Save action is not implemented—button currently logs a TODO.

## Adding/Changing Features
- Adding thermal modes: update `ThermalMode` enum, append to `MODE_CONFIGS`, adjust `NUM_MODES`; ensure names and ranges are sensible.
- New web endpoints: register in `websBegin`; reuse semaphore pattern (`frameReadySem`/`frameFreeSem`) if accessing `readyBuffer`.
- Calibration/inputs: update `processTouch()` mapping if touch orientation/scale changes; top-bar hit regions are defined via `SAVE_BUTTON_*`, `WIFI_BUTTON_*`, `MODE_BUTTON_*`.
- WiFi presets: extend `WIFI_NETWORKS` but avoid committing real credentials; consider moving to an ignored secrets file (TODO noted in `wifiman.cpp`).
- JPEG quality/performance: adjust quality parameter (0–100) in `encodeRGB565ToJPEG` calls; integrate HW encoder for best results (see placeholder comment in `webs.cpp`).

## Build/Run Notes
- Project is structured as a single Arduino sketch; build/flash with ESP32 support (ESP32-S3 target, PSRAM preferred). No automated tests are present.
- Requires libraries: `Adafruit_MLX90640`, `GFX Library for Arduino` (moononournation's Arduino_GFX), `XPT2046_Touchscreen`, `WebServer`, `WiFi`.
- Helper script: `./scripts/arduino-build.sh [--fqbn <fqbn>] [--port <port>] [--flash]` (defaults to esp32:esp32:esp32s3 with PSRAM enabled).

## Quick Checks
- If web stream is slow/blank, confirm core 0 task started (check Serial), both buffers allocated, and semaphores created; ensure WiFi is connected and not in power-save mode.
- If display is frozen, confirm core 1 loop still runs (Serial) and that `mlx.getFrame`/render still succeed; palette/range math expects non-NaN values (see `fixNaNsAndComputeRange()`).
- Low streaming FPS: integrate HW JPEG encoder (replace placeholder in `webs.cpp`); check Wi-Fi throughput (disable power save with `WiFi.setSleep(false)`); verify dual-core task distribution in Serial output.
