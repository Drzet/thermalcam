# Third-party notices

The ILI9488 register initialization and rotation/inversion settings in
main/board.c follow Arduino_GFX's Arduino_ILI9488.h and
Arduino_ILI9488_18bit.cpp, retaining the original sketch's configuration.
Source: https://github.com/moononournation/Arduino_GFX
Reference revision: c2e39350ff09c3720702bd3f050cb581fa45a8e9.
The corresponding BSD license is included as Arduino_GFX-LICENSE.txt.

The Melexis reference calculations and Apache 2.0 license are under
components/mlx90640; UPSTREAM.md records their origin and local change.

Espressif's USB UVC component and TinyUSB are fetched by the ESP-IDF component
manager. Their exact versions and integrity hashes are in dependencies.lock;
the packages include their respective license notices.
