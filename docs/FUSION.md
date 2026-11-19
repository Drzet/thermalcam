# ESP32-P4 OV5647 + MLX90640 fusion branch

Firmware integration has been compiled in ESP-IDF 6.0.3 CI, but the combined
camera pipeline and optical registration have not been hardware-tested. It preserves the original phone-connected, high-speed USB MJPEG UVC
path and the established MLX90640 acquisition/calculation and temperature modes.

## Hardware and provenance

- WT9932P4-TINY camera connector J2: Raspberry Pi-compatible 15-pin MIPI CSI.
- OV5647 (Raspberry Pi Camera v1.x) sensor driver: Espressif `esp_cam_sensor`
  via `esp_video` (not a locally written register-level camera driver).
- Camera SCCB SDA GPIO7, SCL GPIO8, reset/powerdown -1, as specified by
  Wireless-Tag WT9932P4-TINY board implementation. MLX90640 remains on 41/42.
- OV5647 MIPI RAW8 800x640 sensor mode, 2 lanes. Espressif's ISP demosaics
  into RGB565, exposed via V4L2 `/dev/video0` with two mapped frame buffers.
- Existing P4 JPEG encoder, USB UVC callbacks and phone powering are retained.
- No SPI screen or LVGL dependency is introduced.

Upstream:
- [Wireless-Tag WT9932P4-TINY camera example](https://github.com/wireless-tag-com/WT9932P4-TINY/tree/main/video_lcd_display)
- [Espressif esp_video](https://github.com/espressif/esp-video-components/tree/master/esp_video)
- [Original S3 registration/fusion project](https://github.com/my-short-circuits/multicamera_thermal)

## Frame pipeline

```
OV5647 -> CSI -> ISP -> V4L2 RGB565 800x640
                    |
                    v
       P4 PPA: centre crop 768x512, scale 5/8 -> RGB565 480x320
                    |
MLX90640 32x24 (8 complete frames/s)
  -> established float temperatures / palette ranges
  -> bilinear *temperature* interpolation
  -> registered 480x320 ARGB layer (transparent outside optical projection)
                    |
                    v
         P4 PPA per-pixel-alpha BLEND
                    |
                    v
         RGB565 -> BGR888
                    |
                    v
        existing hardware JPEG encoder
                    |
                    v
     existing 480x320 MJPEG USB UVC callbacks -> phone
```

The input crop is mathematically 3:2, so the scaling factors are **0.625**
in both directions and exactly representable in PPA's 1/16-step scaler.

The overlay is cached and regenerated only on new thermal frames, changes of
thermal range mode, or stale-state transitions. Visible video may refresh at
30 frames/s even when thermal temperatures refresh at 8 frames/s.

Outside the configured thermal projection, alpha is zero, leaving the visible
image unchanged. If the MLX90640 is invalid or stale, the thermal layer is
transparent. If CSI initialisation fails, the **original thermal-only UVC
renderer** is used.

The default ARGB alpha is 100/255 (~39%), not additive brightness. The
preexisting BODY temperature mapping remains 30–45°C with Turbo; AUTO and
HOME remain unchanged. The S3's **registration/opacity concept** was adapted;
its palette, automatic range logic, UI, and interpolated *palette-index*
algorithm were deliberately not ported.

## Alignment

The S3 project's **manual-distance parallax, visible-image crop and zoom
mechanism** has been ported. The five S3 presets, distance interval 5..500 cm,
initial 30 cm, reciprocal horizontal shift, constant Y correction and
independent X/Y zoom are preserved. Working distance can be adjusted at
runtime using USB UVC contrast, or through an optional ADC potentiometer
with a chosen GPIO. No ADC pin is assigned by default.

See [ALIGNMENT.md](ALIGNMENT.md) for the exact equation, controls, physical
wiring, calibration parameters and instructions. The original S3 numerical
constants are **not a calibrated optical match** for the P4's OV5647 lens
and MLX90640 placement.

## Building

Use ESP-IDF **v6.0.3**, matching the verified I3C-based thermal baseline. The
branch adds `espressif/esp_video: 2.3.*`, its OV5647 sensor integration, and
P4 PPA. The managed component is responsible for sensor configuration,
CSI capture and ISP setup.

```sh
idf.py set-target esp32p4
idf.py reconfigure build
```

If reusing an old local `sdkconfig`, verify that the effective settings
include `CONFIG_CAMERA_OV5647=y`, the 800x640 RAW8 mode,
`CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER=y`, and
`CONFIG_UVC_CAM1_FRAMERATE=30`. Changing `sdkconfig.defaults` alone does
not override existing values in an old `sdkconfig`.

Host overlay test from the repository root:

```sh
cc -std=c11 -O2 tests/test_fusion_overlay.c -lm -o /tmp/fusion_test
/tmp/fusion_test
```

## Validation still required

ESP-IDF 6.0.3 cross-compilation runs in GitHub Actions. The camera hardware
has **not been flashed or tested**. Validate CSI acquisition, ISP colour order,
PPA buffer/cache alignment, analogue potentiometer input, UVC contrast
control, thermal alignment, actual fused framerate and CPU cost of the
additional S3 visible-image crop.

The existing thermal-only branch was not modified.
