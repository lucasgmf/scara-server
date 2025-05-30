#include "encoder.h"

esp_err_t encoder_init(encoder_t *encoder) {
  if (!encoder || !encoder->i2c_master || !encoder->i2c_slave) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!encoder->i2c_master->bus_handle || !encoder->i2c_slave->dev_cfg ||
      !encoder->i2c_slave->dev_handle) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(encoder->label, "was initialized!");

  // Add encoder device to the bus
  return i2c_master_bus_add_device(*encoder->i2c_master->bus_handle,
                                   encoder->i2c_slave->dev_cfg,
                                   encoder->i2c_slave->dev_handle);
}

uint16_t encoder_read_angle(encoder_t *encoder) {
  uint8_t reg = encoder->reg_angle_msb;
  uint8_t data[2];
  esp_err_t ret;

  if (!encoder || !encoder->i2c_master || !encoder->i2c_master->i2c_mutex) {
    ESP_LOGE("encoder",
             "Invalid encoder pointer or uninitialized i2c_master/i2c_mutex");
    return 0xFFFF;
  }

  if (xSemaphoreTake(encoder->i2c_master->i2c_mutex,
                     encoder->i2c_master->timeout_ticks) != pdTRUE) {
    ESP_LOGW(encoder->label, "Failed to take I2C mutex");
    return 0xFFFF;
  }

  // ---- I2C access section begins ----
  do {
    ret =
        tca_select_channel(encoder->tca_channel, encoder->i2c_tca->dev_handle);
    if (ret != ESP_OK) {
      ESP_LOGE(encoder->label, "Failed to select TCA channel: %s",
               esp_err_to_name(ret));
      break; // Go to cleanup
    }

    ret = i2c_master_transmit_receive(*encoder->i2c_slave->dev_handle, &reg, 1,
                                      data, 2,
                                      encoder->i2c_master->timeout_ticks);
    if (ret != ESP_OK) {
      ESP_LOGE(encoder->label, "Read error: %s", esp_err_to_name(ret));
      break; // Go to cleanup
    }

    // Success
    uint16_t raw = ((data[0] << 8) | data[1]) & encoder->reg_angle_mask;
    if (encoder->reverse)
      raw = encoder->reg_angle_mask - raw;
    raw = (raw + (uint16_t)encoder->offset) & encoder->reg_angle_mask;

    encoder->current_reading = raw;

    xSemaphoreGive(encoder->i2c_master->i2c_mutex);
    return raw;

  } while (0);

  // ---- I2C access failed ----
  xSemaphoreGive(encoder->i2c_master->i2c_mutex);
  return 0xFFFF;
}
