#include "loadcell.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "HX711";

// Global sensor data instance
hx711_data_t g_hx711_data = {0}; /**
                                  * Initialize HX711 driver
                                  */
esp_err_t hx711_init(hx711_config_t *config) {
  if (config == NULL) {
    ESP_LOGE(TAG, "Config is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  // Configure data pin as input
  gpio_config_t data_config = {
      .pin_bit_mask = (1ULL << config->data_pin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret = gpio_config(&data_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure data pin");
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
    ESP_LOGE(TAG, "Failed to configure clock pin");
    return ret;
  }

  // Initialize clock pin to LOW
  gpio_set_level(config->clock_pin, 0);

  // Power up the HX711 (ensure clock is low)
  vTaskDelay(pdMS_TO_TICKS(100));

  // Initialize global data structure
  memset(&g_hx711_data, 0, sizeof(hx711_data_t));

  // Check initial pin states for diagnostics
  int data_level = gpio_get_level(config->data_pin);
  int clock_level = gpio_get_level(config->clock_pin);

  ESP_LOGI(TAG, "HX711 initialized - Data pin: %d, Clock pin: %d", data_level,
           clock_level);
  ESP_LOGI(TAG, "Waiting for HX711 to be ready...");

  // Wait up to 5 seconds for HX711 to be ready
  int ready_timeout = 5000;
  while (!hx711_is_ready(config) && ready_timeout > 0) {
    vTaskDelay(pdMS_TO_TICKS(10));
    ready_timeout -= 10;
  }

  if (ready_timeout <= 0) {
    ESP_LOGE(TAG, "HX711 not ready after 5 seconds. Check wiring and power!");
    ESP_LOGE(TAG, "Data pin should be LOW when ready. Current level: %d",
             gpio_get_level(config->data_pin));
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(TAG, "HX711 is ready!");

  // Do a test read to verify communication
  ESP_LOGI(TAG, "Performing test read...");
  int32_t test_read = hx711_read_raw(config);
  ESP_LOGI(TAG, "Test read result: %ld", test_read);

  return ESP_OK;
}

/**
 * Check if HX711 is ready for reading
 */
bool hx711_is_ready(hx711_config_t *config) {
  return (gpio_get_level(config->data_pin) == 0);
}

/**
 * Read raw value from HX711
 */
int32_t hx711_read_raw(hx711_config_t *config) {
  // Wait for HX711 to be ready
  int timeout = 1000; // 1 second timeout
  while (!hx711_is_ready(config) && timeout > 0) {
    vTaskDelay(pdMS_TO_TICKS(1));
    timeout--;
  }

  if (timeout == 0) {
    ESP_LOGW(TAG, "HX711 not ready, timeout occurred. Data pin level: %d",
             gpio_get_level(config->data_pin));
    return -1; // Return -1 to indicate error
  }

  int32_t data = 0;

  // Disable interrupts during bit-banging for timing accuracy
  portDISABLE_INTERRUPTS();

  // Read 24 bits of data
  for (int i = 0; i < 24; i++) {
    gpio_set_level(config->clock_pin, 1);
    vTaskDelay(1); // Small delay for clock high
    data = (data << 1) | gpio_get_level(config->data_pin);
    gpio_set_level(config->clock_pin, 0);
    vTaskDelay(1); // Small delay for clock low
  }

  // Set gain by sending additional clock pulses
  for (int i = 0; i < config->gain; i++) {
    gpio_set_level(config->clock_pin, 1);
    vTaskDelay(1);
    gpio_set_level(config->clock_pin, 0);
    vTaskDelay(1);
  }

  portENABLE_INTERRUPTS();

  // Convert from 24-bit two's complement to 32-bit signed integer
  if (data & 0x800000) {
    data |= 0xFF000000;
  }

  return data;
}

/**
 * Read average value over multiple samples
 */
int32_t hx711_read_average(hx711_config_t *config, uint8_t samples) {
  if (samples == 0) {
    samples = 1;
  }

  int64_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += hx711_read_raw(config);
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between readings
  }

  return (int32_t)(sum / samples);
}

/**
 * Tare the scale (set zero point)
 */
void hx711_tare(hx711_config_t *config, uint8_t samples) {
  ESP_LOGI(TAG, "Taring scale...");
  config->offset = hx711_read_average(config, samples);
  ESP_LOGI(TAG, "Tare complete. Offset: %ld", config->offset);
}

/**
 * Calibrate the scale with a known weight
 */
void hx711_calibrate(hx711_config_t *config, float known_weight_grams,
                     uint8_t samples) {
  ESP_LOGI(TAG, "Calibrating with %.2f grams...", known_weight_grams);

  // Read current value with the known weight
  int32_t reading = hx711_read_average(config, samples);

  // Calculate scale factor
  int32_t raw_weight = reading - config->offset;
  if (raw_weight != 0) {
    config->scale = known_weight_grams / (float)raw_weight;
    ESP_LOGI(TAG, "Calibration complete. Scale factor: %.6f", config->scale);
  } else {
    ESP_LOGE(TAG, "Calibration failed: no weight detected");
  }
}

/**
 * Get weight in grams
 */
float hx711_get_weight(hx711_config_t *config) {
  int32_t raw = hx711_read_raw(config);
  return (float)(raw - config->offset) * config->scale;
}

/**
 * Power down HX711
 */
void hx711_power_down(hx711_config_t *config) {
  gpio_set_level(config->clock_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(1));
}

/**
 * Power up HX711
 */
void hx711_power_up(hx711_config_t *config) {
  gpio_set_level(config->clock_pin, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
}

/**
 * HX711 reading task
 */
void hx711_task(void *pvParameters) {
  hx711_config_t *config = (hx711_config_t *)pvParameters;

  ESP_LOGI(TAG, "HX711 task started");

  while (1) {
    if (hx711_is_ready(config)) {
      // Read raw value
      int32_t raw_reading = hx711_read_raw(config);

      // Only update if we got a valid reading
      if (raw_reading != -1) {
        g_hx711_data.raw_value = raw_reading;

        // Calculate weight in grams
        g_hx711_data.weight_grams =
            (float)(g_hx711_data.raw_value - config->offset) * config->scale;

        // Calculate weight in kg
        g_hx711_data.weight_kg = g_hx711_data.weight_grams / 1000.0f;

        // Update timestamp
        g_hx711_data.last_update_ms = esp_timer_get_time() / 1000;

        // Mark data as ready
        g_hx711_data.data_ready = true;

        ESP_LOGD(TAG, "Raw: %ld, Weight: %.2f g, %.3f kg",
                 g_hx711_data.raw_value, g_hx711_data.weight_grams,
                 g_hx711_data.weight_kg);
      } else {
        ESP_LOGW(TAG, "Failed to read from HX711");
      }
    } else {
      ESP_LOGD(TAG, "HX711 not ready, data pin level: %d",
               gpio_get_level(config->data_pin));
    }

    // Delay between readings (adjust as needed)
    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz reading rate
  }
}
