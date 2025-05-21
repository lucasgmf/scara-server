#include "encoder_d.h"

uint16_t encoder_read_angle(encoder_t *encoder) {
  uint8_t reg = encoder->settings->angle_reg;
  uint8_t data[2];

  if (xSemaphoreTake(*encoder->i2c_master->i2c_mutex,
                     encoder->i2c_master->timeout_ticks) != pdTRUE) {
    ESP_LOGE(encoder->label, "Mutex timeout");
    return 0xFFFF;
  }

  esp_err_t ret =
      i2c_master_transmit_receive(*encoder->i2c_slave->dev_handle, &reg, 1,
                                  data, 2, encoder->i2c_master->timeout_ticks);

  xSemaphoreGive(*encoder->i2c_master->i2c_mutex);

  if (ret != ESP_OK) {
    ESP_LOGE(encoder->label, "Read error: %s", esp_err_to_name(ret));
    return 0xFFFF;
  }

  uint16_t raw = ((data[0] << 8) | data[1]) & encoder->settings->angle_mask;
  if (encoder->settings->reverse)
    raw = encoder->settings->angle_mask - raw;
  raw = (raw + encoder->settings->zero_offset) & encoder->settings->angle_mask;

  /* ESP_LOGI(encoder->label, "Angle: %u", raw); */
  return raw;
}
