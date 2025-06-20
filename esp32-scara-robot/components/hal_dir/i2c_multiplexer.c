#include "i2c_multiplexer.h"

esp_err_t tca_select_channel(uint8_t channel,
                             i2c_master_dev_handle_t *tca_handle) {
  if (*tca_handle == NULL) {
    ESP_LOGE("tca_select_channel", "Invalid I2C handle");
    return ESP_ERR_INVALID_ARG;
  }

  if (channel > 7) {
    ESP_LOGE("tca_select_channel", "Invalid channel argument: %d", channel);
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data = 1 << channel;
  ESP_LOGD("tca_select_channel", "Selecting channel %d (data = 0x%02X)",
           channel, data);

  esp_err_t err = i2c_master_transmit(*tca_handle, &data, 1, portMAX_DELAY);
  if (err != ESP_OK) {
    ESP_LOGE("tca_select_channel", "I2C transmit failed: %s",
             esp_err_to_name(err));
  }
  return err;
}
