#include "i2c_bus.h"

void init_i2c_master(i2c_configuration_t *i2c_configuration) {
  if (i2c_configuration == NULL ||
      i2c_configuration->as5600_dev_handle_t == NULL ||
      i2c_configuration->dev_cfg_t == NULL ||
      i2c_configuration->bus_handle_t == NULL ||
      i2c_configuration->i2c_mst_config_t == NULL) {
    ESP_LOGW("init_i2c_master", "Pointer is null\n");
    return;
  }

  ESP_ERROR_CHECK(i2c_new_master_bus(i2c_configuration->i2c_mst_config_t,
                                     &i2c_configuration->bus_handle_t));

  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      i2c_configuration->bus_handle_t, i2c_configuration->dev_cfg_t,
      &i2c_configuration->as5600_dev_handle_t));
  ESP_LOGI("init_i2c_master", "init_i2c_master returned without errors");
  return;
}
