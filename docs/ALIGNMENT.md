# S3 manual-distance parallax model on ESP32-P4

This is a **port of the original S3 registration calculations**, not a
replacement optical model. See
[my-short-circuits/multicamera_thermal](https://github.com/my-short-circuits/multicamera_thermal/blob/main/multicamera_thermal.ino)
for `parallaxX100ForDistance`, `recomputeAlignmentTransform`, and
`cameraCropParamsFor`.

The P4 image is 480×320; the S3's pixel offsets remain defined in its
320×240 reference coordinates and are rescaled when cropping the visible
image. The thermal colour layer remains fixed.

## Original S3 mechanism retained

- Distance is **selected manually**, not measured.
- Five named presets: 5 cm, 15 cm, 30 cm, 100 cm, 500 cm.
- A continuous integer distance between 5 and 500 cm, default 30 cm.
- X displacement in S3 reference pixels:
  `px100 = 125 + round(35654 / Z_cm) + user_offset_x100`
- Fixed Y displacement: `py100 = -670 + user_offset_y100`.
- Initial visible zoom: 126% in X and Y.
- Additional horizontal/vertical zoom adjustments retained.
- Visible-image translation/crop uses the S3 fixed-point calculation and
  nearest-neighbour pixel sampling; the thermal layer stays in place.
- The previous fixed 800×640 -> 480×320 PPA downscale is retained before
  applying the S3 crop; PPA subsequently blends the registered visible
  image with the thermal image.

## Runtime distance controls

**USB UVC:** The existing brightness control still changes thermal display
mode (0=AUTO, 1=BODY, 2=HOME). The standard UVC *contrast* control now
represents distance in cm, 5..500, resolution 1. On a Linux host:

```sh
v4l2-ctl -d /dev/videoX --list-ctrls
v4l2-ctl -d /dev/videoX --set-ctrl=contrast=30
```

USB host apps may choose not to expose the contrast control.

**Optional potentiometer:** `THERMALCAM_ALIGN_POT_GPIO=-1` is the safe
default; no GPIO is allocated. To use a 10 kΩ linear pot, select an available
P4 ADC1 pin GPIO16..GPIO22 in menuconfig *after confirming that pin is
accessible on the actual WT9932P4-TINY board*. Wire pot ends to 3.3 V
and GND and the wiper to the ADC GPIO. The IDF ADC oneshot driver reads
the pot every 100 ms, applies a short smoothing filter, and maps rotation
linearly to distance in centimetres, exactly like the S3 distance slider.
`THERMALCAM_ALIGN_POT_MIN_CM` and `MAX_CM` can restrict that range, and
`THERMALCAM_ALIGN_POT_REVERSE` reverses rotation.

When a potentiometer is configured, it overrides the USB contrast value
during operation.

## Optical calibration

**The S3 constants are not valid physical calibration for our OV5647
without measurement.** They were determined for the S3's optics and
11.6 mm horizontal camera offset. The model depends on the actual visible
FOV and the physical horizontal/vertical sensor separation. The menuconfig
parameters expose the base offsets, zoom and reciprocal coefficient
for calibration to our enclosure. The Y coefficient remains constant,
as it was in the S3: this is not a full geometric 3D model.

The handheld unit can be moved freely, but registration is defined only
on the selected distance plane. No ToF sensor or automatic distance
estimation is required.

## Testing

```sh
cc -std=c11 -O2 tests/test_s3_alignment.c main/alignment_s3.c -o /tmp/s3test
/tmp/s3test
```

Hardware validation of the PPA pipeline, UVC host controls, physical
pot input and optical registration is still required.
