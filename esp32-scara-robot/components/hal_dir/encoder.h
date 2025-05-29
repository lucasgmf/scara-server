#ifndef ENCODER_H
#define ENCODER_H

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"
#include "i2c_multiplexer.h"

#include "switch_h.h"

typedef struct {
  const char *label;
  i2c_master_config_t *i2c_master;
  i2c_slave_bus_params *i2c_slave;
  i2c_slave_bus_params *i2c_tca;
  uint8_t tca_channel;
  uint8_t reg_angle_msb;
  uint16_t reg_angle_mask;
  // additional ...
  float offset; // offset from 0
  bool reverse; // TODO: change this later
  float current_reading;
  int32_t accumulated_steps;
  bool is_inverted;
  bool is_calibrated;
  float gear_ratio;
  int test_offset; // WARN: test
  switch_t *switch_n;
  SemaphoreHandle_t *i2c_mutex;
} encoder_t;

esp_err_t encoder_init(encoder_t *encoder);
uint16_t encoder_read_angle(encoder_t *encoder);

#endif // ENCODER_H
