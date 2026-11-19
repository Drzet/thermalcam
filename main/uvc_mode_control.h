#pragma once
#include <stdint.h>
#include "thermal.h"

/* Keep the USB control range tied to the firmware's real mode enum. */
#define THERMAL_MODE_COUNT MODE_COUNT
/* Explicit reference ensures this object is included from main's static archive. */
void thermalcam_uvc_control_init(void);
uint8_t thermalcam_uvc_get_mode(void);
void thermalcam_uvc_set_mode(uint8_t value);
uint16_t thermalcam_uvc_get_distance(void);
void thermalcam_uvc_set_distance(uint16_t value);
