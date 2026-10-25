#pragma once
#include <stdint.h>
#include "thermal.h"

/* Keep the USB control range tied to the firmware's real mode enum. */
#define THERMAL_MODE_COUNT MODE_COUNT
uint8_t thermalcam_uvc_get_mode(void);
void thermalcam_uvc_set_mode(uint8_t value);
