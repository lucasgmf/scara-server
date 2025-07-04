#include "loadcell.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HX711";

esp_err_t hx711_gpio_init(hx711_t *config) {
  if (config == NULL) {
    ESP_LOGE(TAG, "Config is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  ESP_LOGI(TAG, "Initializing HX711 sensor %c", config->label);

  // Configure data pin as input
  gpio_config_t data_config = {
      .pin_bit_mask = (1ULL << config->data_pin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret;

  ret = gpio_config(&data_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure data pin for sensor %c", config->label);
    return ret;
  }

  // Configure clock pin as output
  gpio_config_t clock_config = {
      .pin_bit_mask = (1ULL << config->clock_pin),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  ret = gpio_config(&clock_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure clock pin for sensor %c", config->label);
    return ret;
  }

  gpio_set_level(config->clock_pin, 0);

  vTaskDelay(10 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "HX711 sensor %c is ready!", config->label);
  return ESP_OK;
}

void hx711_read_raw(hx711_t *hx711) {

  int32_t data = 0;

  // Read 24 bits of data
  for (int i = 0; i < 24; i++) {
    gpio_set_level(hx711->clock_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    data = (data << 1) | gpio_get_level(hx711->data_pin);

    gpio_set_level(hx711->clock_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  // Set gain
  for (int i = 0; i < hx711->gain; i++) {
    gpio_set_level(hx711->clock_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(hx711->clock_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  // Convert from 24-bit two's complement to 32-bit signed integer
  if (data & 0x800000) {
    data |= 0xFF000000;
  }

  hx711->raw_read = data;
  return;
}
