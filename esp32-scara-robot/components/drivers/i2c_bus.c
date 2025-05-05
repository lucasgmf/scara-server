#include "i2c_bus.h"

void init_i2c_master(i2c_master_bus_config_t *i2c_mst_config_t,
                     i2c_device_config_t *dev_cfg_t,
                     i2c_master_bus_handle_t *bus_handle_t,
                     i2c_master_dev_handle_t *as5600_dev_handle_t) {
  ESP_ERROR_CHECK(i2c_new_master_bus(i2c_mst_config_t, bus_handle_t));

  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(*bus_handle_t, dev_cfg_t, as5600_dev_handle_t));
  ESP_LOGI("init_i2c_master", "init_i2c_master returned without errors");
  return;
}
