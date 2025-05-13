#ifndef ENCODER_H
#define ENCODER_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "i2c_bus.h"
#include "switch_h.h"

typedef struct {
  int id;
  bool is_calibrated;
  uint16_t raw_val;
  uint32_t offset;
  switch_t *cal_switch;
  i2c_configuration_t *i2c_configuration;
} mag_encoder;

uint16_t get_as5600_reading(mag_encoder *mag_encoder_n, uint8_t msb_reg_angle);

#endif // ENCODER_H
