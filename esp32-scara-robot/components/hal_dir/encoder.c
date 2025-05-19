#include "encoder.h"

// Init function
void init_encoder(encoder_conf *conf) {
  ESP_ERROR_CHECK(i2c_new_master_bus(conf->i2c_mst_config, conf->bus_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(*conf->bus_handle, conf->dev_cfg,
                                            conf->as5600_dev_handle));
}
