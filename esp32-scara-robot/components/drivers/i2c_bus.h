#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
  i2c_master_bus_config_t *bus_cfg;
  i2c_master_bus_handle_t *bus_handle;
  SemaphoreHandle_t i2c_mutex;
  TickType_t timeout_ticks;
} i2c_master_config_t;

typedef struct {
  i2c_device_config_t *dev_cfg;
  i2c_master_dev_handle_t *dev_handle;
} i2c_slave_bus_params;

#endif // I2C_BUS_H
