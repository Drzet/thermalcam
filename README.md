# ThermalCam P4 — USB thermal webcam

ESP-IDF firmware for Wireless-Tag WT9932P4-TINY and MLX90640. Replaces the
Arduino/HTTP implementation with a 480×320 MJPEG UVC device, while retaining
the 480×320 ILI9488 SPI display, XPT2046 touch and physical mode button.
The original S3 implementation remains in Git history and on main.

## Hardware and build baseline

- ESP-IDF **v5.5.1**, target **esp32p4**. This SDK supports P4 revisions v0.1
  through v1.99 with the supplied defaults; **P4X/revision v3.x is not supported
  by this build**. Confirm the board revision before flashing.
- The flash layout uses 2 MiB; this is a build setting, not a measurement of the board capacity.
- PSRAM required. Configured for 80 MHz; capacity is detected at boot.
- `espressif/usb_device_uvc` **1.3.1**; transitive dependencies pinned by
  `dependencies.lock`.
- UVC uses the **high-speed USB OTG port**; the separate USB Serial/JTAG
  controller is the console. On WT9932P4-TINY use HUSB for video, FUSB for
  flashing/debugging, after confirming these labels against your board revision.
- Display SPI: 40 MHz. Touch SPI: 2 MHz. Sensor I2C: **400 kHz**, within the
  sensor specification; external I2C pull-ups are still required.

The previous GPIO assignments are retained by user instruction, not inferred
from a board pinout. All assignments are in `main/board.h`:

| Function | GPIO |
|---|---|
| TFT BL / DC / CS / reset | 16 / 19 / 39 / 38 |
| TFT SCK / MOSI / MISO | 15 / 7 / 17 |
| Touch CLK / CS / DIN / DO / IRQ | 10 / 11 / 12 / 13 / 14 |
| Physical mode button (active low) | 3 |
| MLX90640 SDA / SCL | 41 / 42 |

## Build

With the ESP-IDF v5.5.1 environment activated:

```sh
idf.py set-target esp32p4
idf.py build
```

Use the resulting build's flash arguments and the correct serial device for
your connected board. No board was flashed during development.

## Behaviour

- Advertises only **480×320 MJPEG at 4 fps**. The sensor is configured for
  8 subpages/s; a complete thermal frame needs both subpages. The 4 fps USB
  presentation rate is not a promise of four distinct measured frames each
  second; measured age is displayed.
- The thermal view remains 384×288, interpolated from 32×24 measurements.
  Upscaling does not add thermal resolution. The original cold-white/hot-black
  palette is retained.
- Top left: **Freeze/Resume**. Top right and physical button: cycle **Auto**,
  **Body (30–45°C)** and **Home (-20–100°C)**. Centre: status, not Wi-Fi.
- Sidebar shows measured extrema, frame age, repaired pixel count and colour
  scale values. Measured extrema are not widened to create the display span.
- The old touch orientation is preserved; raw endpoint calibration remains a
  physical-panel check. Coordinates are bounded and presses/releases debounced.
- USB video includes labels and errors. It does **not** transport radiometric
  floats; retain a separate data interface as a future extension if needed.
- No Wi-Fi manager, HTTP server or stored network credentials in this version.
  Credentials in earlier public commits remain exposed; deleting current files
  does not revoke them or rewrite history.

## Acquisition and concurrency

The sensor task owns the I2C bus and Melexis calibration state. It verifies
configuration readback and waits for both subpages with a 750 ms deadline,
yielding while waiting. Failed or overlapped reads never replace the last valid
frame. Three consecutive acquisition errors trigger sensor reinitialization;
a missing sensor is retried once per second without stopping USB/UI.

Factory bad pixels are invalidated before correction. At most four invalid
pixels are repaired, using only finite neighbours from the original frame.
A pixel with no valid original neighbours, or a frame with too many invalid
pixels, rejects the whole frame. Infinity is treated as invalid, just like NaN.

Emissivity remains **0.95**, with reflected temperature estimated as sensor
ambient minus **8°C**, matching the Adafruit calculation previously used.
Body mode changes the colour scale only, not calibration or emissivity.

Acquisition, input and rendering are independent tasks. Rendering takes a short
snapshot of shared state, produces BGR888 in JPEG-compatible memory, encodes
with the P4 hardware encoder, then publishes a complete JPEG under a mutex.
The UVC driver copies the published JPEG before releasing it; it never holds
the sensor lock during USB transfers. A slow/disconnected host cannot block
sensor acquisition or touch polling. The local display and USB share the same
rendered view. JPEG failure withdraws the published frame instead of silently
serving stale video. A stale sensor frame is explicitly marked; intentional
freeze is separately labelled and retains its original acquisition age.

## Host regression test

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I main tests/test_thermal.c main/thermal.c -lm -o /tmp/thermalcam-test
/tmp/thermalcam-test
```

An optional argument writes a PPM preview. In containers where LeakSanitizer
cannot inspect `/proc`, run with `ASAN_OPTIONS=detect_leaks=0`; address and
undefined-behaviour checks remain enabled.

## Hardware acceptance checks

1. Confirm chip revision, GPIO wiring, HUSB/FUSB routing and touch alignment.
2. Open the UVC device on the intended host; verify negotiated 480×320 MJPEG,
   colours matching the LCD, temperatures and age updating.
3. Change modes and freeze/resume both while streaming and without a host.
4. Disconnect the sensor: stale/no-sensor indication must appear, input and USB
   must remain responsive. Reconnect and verify recovery.
5. Close/reopen the host video application and unplug/replug USB repeatedly.
6. Compare temperature readings with the previous implementation on the same
   scene and check that actual fresh-frame cadence is acceptable.

See `third_party/NOTICE.md` and `components/mlx90640/UPSTREAM.md` for provenance.
