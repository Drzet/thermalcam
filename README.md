# ThermalCam P4 — USB thermal webcam

ESP-IDF firmware for Wireless-Tag WT9932P4-TINY and MLX90640. Replaces the
Arduino/HTTP implementation with a phone-connected 480×320 MJPEG UVC device.
The physical mode button is retained; no local display or touchscreen is used.
The original S3 implementation remains in Git history and on main.

## Hardware and build baseline

- Experimental branch uses **ESP-IDF v6.0.3**, target **esp32p4**,
  matching the verified running P4 build. The board's confirmed **v1.0**
  silicon revision is pinned via `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`
  and `CONFIG_ESP32P4_REV_MIN_100=y` in `sdkconfig.defaults` (IDF 6
  otherwise defaults to incompatible v3.1). It relies on the v6.0.3
  `esp_driver_i3c` legacy-I2C API. In v6.0.3 this driver is gated by
  `CONFIG_IDF_EXPERIMENTAL_FEATURES=y` (which enables the hidden
  `CONFIG_I3C_MASTER_ENABLED=y`); the setting is included in
  `sdkconfig.defaults`. If a local `sdkconfig` exists, enable **Make
  experimental features visible** in `idf.py menuconfig` before building.
- The flash layout uses 2 MiB; this is a build setting, not a measurement of the board capacity.
- PSRAM required. Configured for 80 MHz; capacity is detected at boot.
- `espressif/usb_device_uvc` **1.3.1**; transitive dependencies pinned by
  `dependencies.lock`.
- UVC uses the **high-speed USB OTG port**; the separate USB Serial/JTAG
  controller is the console. On WT9932P4-TINY use HUSB for video, FUSB for
  flashing/debugging, after confirming these labels against your board revision.
- **Experimental sensor transport:** P4 I3C master in legacy 7-bit I2C
  mode at **1 MHz** for RAM/register traffic and **400 kHz** for EEPROM.
  Uses the same GPIO41 SDA / GPIO42 SCL; no wiring change in software.
  The I3C legacy-I2C driver does not support clock-stretching slaves.
  GPIO41/42 have no dedicated I3C programmable pull-ups, so adequate
  external pull-ups are necessary for 1 MHz.

The previous GPIO assignments are retained by user instruction, not inferred
from a board pinout. All assignments are in `main/camera_pins.h`:

| Function | GPIO |
|---|---|
| Physical mode button (active low) | 3 |
| MLX90640 SDA / SCL | 41 / 42 |

## Build

With the ESP-IDF v6.0.3 environment activated:

```sh
idf.py set-target esp32p4
idf.py build
```

`set-target` is a **one-time** regeneration for existing checkouts with an old
local `sdkconfig`; it backs up the old config, clears build artifacts and
applies this branch's tracked defaults. Normal subsequent builds only need
`idf.py build`. Verify `sdkconfig` has both v1.0 revision selections
and `CONFIG_I3C_MASTER_ENABLED=y`. The project never edits local `sdkconfig`
when switching branches, so changing `sdkconfig.defaults` alone does not
override values retained in an existing file.

Use the resulting build's flash arguments and the correct serial device for
your connected board. No board was flashed during development.

## Behaviour

- Advertises **480×320 MJPEG at 32 fps**, matching the theoretical upper
  bound for 64 sensor subpages/s (two subpages per complete 32×24 thermal measurement).
  USB presentation rate is distinct from the rate of fresh thermal frames;
  any shortage is visible in the five-second acquisition statistics.
  Existing local `sdkconfig` files must be regenerated or adjusted after
  changing `sdkconfig.defaults` to ensure the UVC descriptor changes
  (`CONFIG_UVC_CAM1_FRAMERATE=32`).
- The thermal view remains 384×288, interpolated from 32×24 measurements.
  Upscaling does not add thermal resolution. The original cold-white/hot-black
  palette is retained.
- Physical button on **GPIO3 to GND**: cycle **Auto**, **Body (30–45°C)**
  and **Home (-20–100°C)**, with 50 ms debounce and one change per press.
  The USB image shows the active mode and camera status; labels are not touch controls.
- Sidebar shows measured extrema, frame age, repaired pixel count and colour
  scale values. Measured extrema are not widened to create the display span.
- USB video includes labels and errors. It does **not** transport radiometric
  floats; retain a separate data interface as a future extension if needed.
- No Wi-Fi manager, HTTP server or stored network credentials in this version.
  Credentials in earlier public commits remain exposed; deleting current files
  does not revoke them or rewrite history.

## Acquisition and concurrency

The sensor task owns the I3C legacy-I2C bus and configuration. It switches
to the 400 kHz handle before every EEPROM dump, then to the 1 MHz handle
after sensor configuration and a fast-register readback. It verifies
configuration readback and waits for two consecutive, opposite subpages with
a 750 ms deadline, yielding while waiting. The sensor uses interleaved (TV)
mode and reads only the twelve newly updated 32-pixel rows per subpage; the
64 auxiliary words are still read and validated. Repeated, tardy or overlapped
subpages never replace the last valid frame. The acquisition path accepts a
newly ready *opposite* subpage after the current pixel read because the two
subpages use disjoint rows in TV mode. It rejects a same-subpage overwrite.
Auxiliary words are read before the pixel rows. The I2C task copies each
complete raw subpage pair into a nonblocking queue and immediately resumes
readout. A separate calculation task processes radiometric temperatures, checks
pixels and publishes fresh frames. A queue overflow discards a raw pair; it
never stalls the I2C task. Repeated or missed subpages do not reconfigure the
sensor. Three consecutive I2C/control failures do trigger reconfiguration;
a missing sensor is retried once per second without stopping USB/UI.
The five-second log reports completed full-frame fps, rejected pairs, queue
drops, maximum pair-read time and calculation time.

Factory bad pixels are invalidated before correction. At most four invalid
pixels are repaired, using only finite neighbours from the original frame.
A pixel with no valid original neighbours, or a frame with too many invalid
pixels, rejects the whole frame. Infinity is treated as invalid, just like NaN.

Emissivity remains **0.95**, with reflected temperature estimated as sensor
ambient minus **8°C**, matching the Adafruit calculation previously used.
Body mode changes the colour scale only, not calibration or emissivity.

Acquisition, temperature calculation, input and rendering are independent tasks.
Rendering takes a short
snapshot of shared state, produces BGR888 in JPEG-compatible memory, encodes
with the P4 hardware encoder, then publishes a complete JPEG under a mutex.
The UVC driver copies the published JPEG before releasing it; it never holds
the sensor lock during USB transfers. A slow/disconnected host cannot block
sensor acquisition or button polling. JPEG failure withdraws the published
frame instead of silently serving stale video. A stale sensor frame is explicitly
marked. Display SPI transfers, touchscreen polling and Freeze/Resume are removed.

## Host regression test

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I main tests/test_thermal.c main/thermal.c -lm -o /tmp/thermalcam-test
/tmp/thermalcam-test
```

An optional argument writes a PPM preview. In containers where LeakSanitizer
cannot inspect `/proc`, run with `ASAN_OPTIONS=detect_leaks=0`; address and
undefined-behaviour checks remain enabled.

## Experimental 1 MHz / 64 Hz qualification

This branch is NOT the known-good 400 kHz / 32 Hz configuration. The latter
is preserved at `feature/p4-usb-thermal-camera` (15.6 fps, 0 rejects/drops
measured). No hardware test of the new I3C transport has yet been performed.

1. Verify the selected ESP-IDF version is 6.0.3 and set the local `sdkconfig`
   UVC frame rate to 32 (or regenerate it), then rebuild.
2. On boot, EEPROM and configuration are accessed at 400 kHz; register/RAM
   reads switch to 1 MHz. Check the startup log and the first acquisition
   lines for transport failures and rejected pairs.
3. Record three consecutive five-second `Acquisition` reports: sustained
   **fresh** complete fps, rejected frames, queue drops, maximum raw-pair read
   time and maximum calculation time. At 64 Hz, 32 complete fps is the
   theoretical ceiling, not a promise.
4. Check image correctness and USB video independently. Existing Android
   clients may report display FPS rather than fresh sensor frames.
5. If the experimental build cannot communicate with the sensor, switch back
   to the known-good branch. Do not change wiring solely based on a failed
   1 MHz attempt; first establish the electrical timing with a scope or
   logic analyser.

## Hardware acceptance checks

1. Confirm chip revision, GPIO wiring, HUSB/FUSB routing and button wiring.
2. Open the UVC device on the intended host; verify negotiated 480×320 MJPEG at 32 fps,
   correct colours, temperatures and age updating.
3. Check the serial `Acquisition` reports: verify sustained fresh thermal fps,
   rejection count, dropped-pair count and maximum pair-read/calculation duration
   separately from the advertised 32 fps UVC output.
   The 32 full fps rate is a theoretical upper bound, not hardware-verified.
4. Press/release and hold the button: one mode change per press. Verify
   the mode persists across USB disconnect/reconnect.
5. Disconnect the sensor: stale/no-sensor indication must appear, input and USB
   must remain responsive. Reconnect and verify recovery.
6. Close/reopen the host video application and unplug/replug USB repeatedly.
7. Compare temperature readings with the previous implementation on the same
   scene and check that actual fresh-frame cadence is acceptable.

See `third_party/NOTICE.md` and `components/mlx90640/UPSTREAM.md` for provenance.
