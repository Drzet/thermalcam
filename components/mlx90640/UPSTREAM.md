# Melexis MLX90640 reference calculation library

Source: https://github.com/melexis/mlx90640-library
Commit: f6be7ca1d4a55146b705f3d347f84b773b29cc86
Files: functions/MLX90640_API.c, headers/*.h, LICENSE.

Local change: GetFrameData returns a failed ValidateAuxData result immediately,
instead of overwriting it with ValidateFrameData and allowing uninitialized
auxiliary readings into the temperature calculation. The ESP-IDF I2C adapter
in main/sensor.c bounds all calls by a capture/configuration deadline and yields
while waiting for data ready. No upstream Linux I2C implementation is used.

Line endings and trailing whitespace are normalized in the vendored source.
