#include "i2c_multiplexer.h"

esp_err_t tca_select_channel(uint8_t channel,
                             i2c_master_dev_handle_t *tca_handle) {
  if (*tca_handle == NULL) {
    ESP_LOGE("tca_select_channel", "Invalid I2C handle");
    return ESP_ERR_INVALID_ARG;
  }

  if (channel > 7) {
    ESP_LOGI("tca_handle", "invalid channel arg");
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data = 1 << channel;
  return i2c_master_transmit(*tca_handle, &data, 1, portMAX_DELAY);
}
