#ifndef I2C_BUS_H
#define I2C_BUS_H
#include "driver/i2c_master.h"
#include "esp_log.h"

typedef struct {
  i2c_master_bus_config_t *i2c_mst_config_t;
  i2c_device_config_t *dev_cfg_t;
  i2c_master_bus_handle_t bus_handle_t;
  i2c_master_dev_handle_t as5600_dev_handle_t;
} i2c_configuration_t;

void init_i2c_master(i2c_configuration_t *i2c_configuration);

#endif // I2C_BUS_H
