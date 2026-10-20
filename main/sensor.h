#pragma once
#include "esp_err.h"
#include "thermal.h"
esp_err_t sensor_bus_init(void);
int sensor_configure(void);
bool sensor_capture(thermal_frame_t *out);
