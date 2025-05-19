#ifndef ENCODER_H
#define ENCODER_H

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"

#include "switch_h.h"

typedef struct {
  uint8_t angle_reg;
  uint16_t angle_mask;
  bool reverse;
  int16_t zero_offset;
  // TODO: Add calibration params etc...
} encoder_settings_t;

typedef struct {
  encoder_settings_t *settings;
  i2c_master_config_t *i2c_master;
  i2c_slave_config_t *i2c_slave;
  const char *label;
} encoder_t;

esp_err_t encoder_init(encoder_t *encoder);

#endif // ENCODER_H
