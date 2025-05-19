#ifndef ENCODER_H
#define ENCODER_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "esp_task.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"
#include "switch_h.h"

typedef struct {
  i2c_master_bus_config_t *i2c_mst_config;
  i2c_master_bus_handle_t *bus_handle;
  i2c_device_config_t *dev_cfg;
  i2c_master_dev_handle_t *as5600_dev_handle;
  uint8_t reg_addr;
  const char *label;
  SemaphoreHandle_t *i2c_mutex;
  TickType_t i2c_timeout_ticks;
} encoder_conf;

void init_encoder(encoder_conf *conf);

#endif // ENCODER_H
