#include "encoder.h"

esp_err_t encoder_init(encoder_t *encoder) {
  esp_err_t ret;

  // Init bus (if not already)
  ret = i2c_new_master_bus(encoder->i2c_master->bus_cfg,
                           encoder->i2c_master->bus_handle);
  if (ret != ESP_OK)
    return ret;

  // Add device
  ret = i2c_master_bus_add_device(*encoder->i2c_master->bus_handle,
                                  encoder->i2c_slave->dev_cfg,
                                  encoder->i2c_slave->dev_handle);
  return ret;
}
