#pragma once
#include <stdint.h>
/* Implemented in app_main.c; both physical button and UVC use one mode. */
#define THERMAL_MODE_COUNT 3
uint8_t thermalcam_uvc_get_mode(void);
void thermalcam_uvc_set_mode(uint8_t value);
