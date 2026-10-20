#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
/* Existing S3 wiring retained by explicit user instruction. */
#define PIN_TFT_BL 16
#define PIN_TFT_DC 19
#define PIN_TFT_CS 39
#define PIN_TFT_RST 38
#define PIN_TFT_SCK 15
#define PIN_TFT_MOSI 7
#define PIN_TFT_MISO 17
#define PIN_TOUCH_CLK 10
#define PIN_TOUCH_CS 11
#define PIN_TOUCH_MOSI 12
#define PIN_TOUCH_MISO 13
#define PIN_TOUCH_IRQ 14
#define PIN_BUTTON 3
#define PIN_SDA 41
#define PIN_SCL 42
esp_err_t board_init(void);
esp_err_t board_draw(const uint8_t *bgr);
bool board_touch(int *x, int *y);
