#ifndef LOADCELL_H
#define LOADCELL_H
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// HX711 gain options
typedef enum {
  HX711_GAIN_128 = 1, // Channel A, gain 128
  HX711_GAIN_32 = 2,  // Channel B, gain 32
  HX711_GAIN_64 = 3   // Channel A, gain 64
} hx711_gain_t;

// HX711 configuration structure
typedef struct {
  const char *label;
  gpio_num_t data_pin;
  gpio_num_t clock_pin;
  hx711_gain_t gain;
  int32_t tare_offset;
  float scale; // Scale factor for calibration
  int32_t raw_read;
} hx711_t;

esp_err_t hx711_gpio_init(hx711_t *config);
void hx711_read_raw(hx711_t *config);

#endif // LOADCELL_H
