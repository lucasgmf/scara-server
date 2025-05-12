#ifndef ENCODER_H
#define ENCODER_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "switch_h.h"

typedef struct {
  int id;
  bool is_calibrated;
  uint16_t raw_val;
  uint32_t offset;
  switch_t *cal_switch;
  i2c_master_bus_handle_t *i2c_mst_config;
  i2c_device_config_t *dev_cfg;
  i2c_master_bus_handle_t *bus_handle;
  i2c_master_dev_handle_t *as5600_dev_handle;
} mag_encoder;

uint16_t get_as5600_reading(i2c_master_dev_handle_t *as5600_dev_handle_t,
                            uint8_t msb_reg_angle);
#endif // ENCODER_H
