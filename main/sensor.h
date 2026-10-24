#pragma once
#include "esp_err.h"
#include "thermal.h"
esp_err_t sensor_bus_init(void);
int sensor_configure(void);
/* Per-attempt timings and precise rejection cause. Sensor task only. */
typedef struct {
    const char *failure;
    uint32_t read_us;        /* Ready wait + I2C read time across subpages. */
    uint32_t ready_wait_us;
    uint32_t calculate_us;
    uint32_t page_gap_us;    /* Between completed subpage reads. */
    unsigned attempts;
    unsigned opposite_ready; /* Ready flag set by following subpage while reading. */
    int last_page;
} sensor_diag_t;
bool sensor_capture(thermal_frame_t *out, sensor_diag_t *diag);
