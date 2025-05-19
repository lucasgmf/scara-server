#include "encoder_d.h"
#include "freertos/semphr.h"

uint16_t read_as5600_angle(encoder_conf *conf) {
  uint8_t angle_data[2];
  esp_err_t ret;

  xSemaphoreTake(*conf->i2c_mutex, conf->i2c_timeout_ticks);

  ret = i2c_master_transmit_receive(*conf->as5600_dev_handle, &conf->reg_addr,
                                    1, angle_data, 2, -1);

  xSemaphoreGive(*conf->i2c_mutex); // 🆕 Release mutex

  if (ret != ESP_OK) {
    ESP_LOGE(conf->label, "Read failed: %s", esp_err_to_name(ret));
    return 0xFFFF;
  }

  uint16_t angle = ((angle_data[0] << 8) | angle_data[1]) & 0x0FFF;
  ESP_LOGI(conf->label, "Angle: %u", angle);
  return angle;
}
