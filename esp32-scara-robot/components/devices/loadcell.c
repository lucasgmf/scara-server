#include "loadcell.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "HX711";

// Global sensor data instances for both sensors
hx711_data_t g_hx711_data[HX711_SENSOR_COUNT] = {0};

// Global command instances
hx711_command_t g_hx711_commands[HX711_SENSOR_COUNT] = {0};

/**
 * Initialize HX711 driver
 */
esp_err_t hx711_init(hx711_config_t *config) {
  if (config == NULL) {
    ESP_LOGE(TAG, "Config is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  if (config->sensor_id >= HX711_SENSOR_COUNT) {
    ESP_LOGE(TAG, "Invalid sensor ID: %d", config->sensor_id);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing HX711 sensor %d", config->sensor_id);

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
    ESP_LOGE(TAG, "Failed to configure data pin for sensor %d",
             config->sensor_id);
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
    ESP_LOGE(TAG, "Failed to configure clock pin for sensor %d",
             config->sensor_id);
    return ret;
  }

  // Initialize clock pin to LOW
  gpio_set_level(config->clock_pin, 0);

  // Power up the HX711 (ensure clock is low)
  vTaskDelay(pdMS_TO_TICKS(100));

  // Initialize global data structure for this sensor
  memset(&g_hx711_data[config->sensor_id], 0, sizeof(hx711_data_t));
  memset(&g_hx711_commands[config->sensor_id], 0, sizeof(hx711_command_t));

  ESP_LOGI(TAG, "Waiting for HX711 sensor %d to be ready...",
           config->sensor_id);

  // Wait up to 5 seconds for HX711 to be ready
  int ready_timeout = 5000;
  while (!hx711_is_ready(config) && ready_timeout > 0) {
    vTaskDelay(pdMS_TO_TICKS(10));
    ready_timeout -= 10;
  }

  if (ready_timeout <= 0) {
    ESP_LOGE(TAG, "HX711 sensor %d not ready after 5 seconds",
             config->sensor_id);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(TAG, "HX711 sensor %d is ready!", config->sensor_id);
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
    vTaskDelay(pdMS_TO_TICKS(1000));
    timeout--;
  }

  if (timeout == 0) {
    ESP_LOGW(TAG, "HX711 sensor %d not ready, timeout occurred",
             config->sensor_id);
    return -1; // Return -1 to indicate error
  }

  int32_t data = 0;

  // Disable interrupts during bit-banging for timing accuracy
  portDISABLE_INTERRUPTS();

  // Read 24 bits of data
  for (int i = 0; i < 24; i++) {
    gpio_set_level(config->clock_pin, 1);
    // Use esp_rom_delay_us for more precise timing
    esp_rom_delay_us(10);
    data = (data << 1) | gpio_get_level(config->data_pin);
    gpio_set_level(config->clock_pin, 0);
    esp_rom_delay_us(10);
  }

  // Set gain by sending additional clock pulses
  for (int i = 0; i < config->gain; i++) {
    gpio_set_level(config->clock_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(config->clock_pin, 0);
    esp_rom_delay_us(10);
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
  int valid_readings = 0;

  for (uint8_t i = 0; i < samples; i++) {
    int32_t reading = hx711_read_raw(config);
    if (reading != -1) {
      sum += reading;
      valid_readings++;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between readings
  }

  if (valid_readings == 0) {
    ESP_LOGE(TAG, "No valid readings for sensor %d", config->sensor_id);
    return -1;
  }

  return (int32_t)(sum / valid_readings);
}

/**
 * Thread-safe tare function
 */
void hx711_tare_safe(hx711_sensor_id_t sensor_id, uint8_t samples) {
  if (sensor_id >= HX711_SENSOR_COUNT) {
    ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
    return;
  }

  ESP_LOGI(TAG, "Requesting tare for sensor %d...", sensor_id);

  // Request tare operation
  g_hx711_commands[sensor_id].tare_requested = true;
  g_hx711_commands[sensor_id].tare_completed = false;
  g_hx711_commands[sensor_id].samples = samples;

  // Wait for taring to complete (max 10 seconds)
  int timeout = 10000;
  while (!g_hx711_commands[sensor_id].tare_completed && timeout > 0) {
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout -= 100;
  }

  if (timeout <= 0) {
    ESP_LOGE(TAG, "Tare timeout for sensor %d", sensor_id);
    g_hx711_commands[sensor_id].tare_requested = false;
  } else {
    ESP_LOGI(TAG, "Tare completed for sensor %d", sensor_id);
  }
}

/**
 * Thread-safe calibrate function
 */
void hx711_calibrate_safe(hx711_sensor_id_t sensor_id, float known_weight_grams,
                          uint8_t samples) {
  if (sensor_id >= HX711_SENSOR_COUNT) {
    ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
    return;
  }

  ESP_LOGI(TAG, "Requesting calibration for sensor %d...", sensor_id);

  // Request calibration operation
  g_hx711_commands[sensor_id].calibrate_requested = true;
  g_hx711_commands[sensor_id].calibrate_completed = false;
  g_hx711_commands[sensor_id].calibration_weight = known_weight_grams;
  g_hx711_commands[sensor_id].samples = samples;

  // Wait for calibration to complete (max 10 seconds)
  int timeout = 10000;
  while (!g_hx711_commands[sensor_id].calibrate_completed && timeout > 0) {
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout -= 100;
  }

  if (timeout <= 0) {
    ESP_LOGE(TAG, "Calibration timeout for sensor %d", sensor_id);
    g_hx711_commands[sensor_id].calibrate_requested = false;
  } else {
    ESP_LOGI(TAG, "Calibration completed for sensor %d", sensor_id);
  }
}

/**
 * Improved HX711 reading task with command handling
 */
void hx711_task(void *pvParameters) {
  hx711_config_t *config = (hx711_config_t *)pvParameters;
  int sensor_id = config->sensor_id;

  ESP_LOGI(TAG, "HX711 task started for sensor %d", sensor_id);

  while (1) {
    // Handle tare request
    if (g_hx711_commands[sensor_id].tare_requested) {
      ESP_LOGI(TAG, "Processing tare request for sensor %d", sensor_id);

      // Temporarily stop normal operation
      g_hx711_data[sensor_id].data_ready = false;

      // Take multiple readings for averaging
      int32_t tare_value =
          hx711_read_average(config, g_hx711_commands[sensor_id].samples);

      if (tare_value != -1) {
        config->offset = tare_value;
        ESP_LOGI(TAG, "Tare complete for sensor %d. Offset: %ld", sensor_id,
                 config->offset);
      } else {
        ESP_LOGE(TAG, "Tare failed for sensor %d", sensor_id);
      }

      // Clear request and mark as completed
      g_hx711_commands[sensor_id].tare_requested = false;
      g_hx711_commands[sensor_id].tare_completed = true;

      // Small delay before resuming normal operation
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Handle calibration request
    if (g_hx711_commands[sensor_id].calibrate_requested) {
      ESP_LOGI(TAG, "Processing calibration request for sensor %d", sensor_id);

      // Temporarily stop normal operation
      g_hx711_data[sensor_id].data_ready = false;

      // Read current value with the known weight
      int32_t reading =
          hx711_read_average(config, g_hx711_commands[sensor_id].samples);

      if (reading != -1) {
        // Calculate scale factor
        int32_t raw_weight = reading - config->offset;
        if (raw_weight != 0) {
          config->scale = g_hx711_commands[sensor_id].calibration_weight /
                          (float)raw_weight;
          ESP_LOGI(TAG,
                   "Calibration complete for sensor %d. Scale factor: %.6f",
                   sensor_id, config->scale);
        } else {
          ESP_LOGE(TAG, "Calibration failed for sensor %d: no weight detected",
                   sensor_id);
        }
      } else {
        ESP_LOGE(TAG, "Calibration failed for sensor %d: read error",
                 sensor_id);
      }

      // Clear request and mark as completed
      g_hx711_commands[sensor_id].calibrate_requested = false;
      g_hx711_commands[sensor_id].calibrate_completed = true;

      // Small delay before resuming normal operation
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Normal reading operation
    if (hx711_is_ready(config)) {
      // Read raw value
      int32_t raw_reading = hx711_read_raw(config);

      // Only update if we got a valid reading
      if (raw_reading != -1) {
        g_hx711_data[sensor_id].raw_value = raw_reading;

        // Calculate weight in grams
        g_hx711_data[sensor_id].weight_grams =
            (float)(raw_reading - config->offset) * config->scale;

        // Calculate weight in kg
        g_hx711_data[sensor_id].weight_kg =
            g_hx711_data[sensor_id].weight_grams / 1000.0f;

        // Update timestamp
        g_hx711_data[sensor_id].last_update_ms = esp_timer_get_time() / 1000;

        // Mark data as ready
        g_hx711_data[sensor_id].data_ready = true;

        ESP_LOGD(TAG, "Sensor %d - Raw: %ld, Weight: %.2f g, %.3f kg",
                 sensor_id, raw_reading, g_hx711_data[sensor_id].weight_grams,
                 g_hx711_data[sensor_id].weight_kg);
      } else {
        ESP_LOGW(TAG, "Failed to read from HX711 sensor %d", sensor_id);
      }
    }

    // Delay between readings
    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz reading rate
  }
}

// Keep your existing functions for backward compatibility
void hx711_tare(hx711_config_t *config, uint8_t samples) {
  hx711_tare_safe(config->sensor_id, samples);
}

void hx711_calibrate(hx711_config_t *config, float known_weight_grams,
                     uint8_t samples) {
  hx711_calibrate_safe(config->sensor_id, known_weight_grams, samples);
}

float hx711_get_weight(hx711_config_t *config) {
  if (g_hx711_data[config->sensor_id].data_ready) {
    return g_hx711_data[config->sensor_id].weight_grams;
  }
  return 0.0f;
}

void hx711_power_down(hx711_config_t *config) {
  gpio_set_level(config->clock_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void hx711_power_up(hx711_config_t *config) {
  gpio_set_level(config->clock_pin, 0);
  vTaskDelay(pdMS_TO_TICKS(1000));
}
