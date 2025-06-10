#include "encoder.h"

#define DEFAULT_ENCODER_RESOLUTION 4096
#define DEGREES_PER_REVOLUTION 360.0f

// Calculate degrees from raw encoder value
float encoder_raw_to_degrees(encoder_t *encoder, uint16_t raw_value) {
  if (!encoder)
    return 0.0f;

  // Convert raw value to degrees (0-360)
  float degrees =
      ((float)raw_value / encoder->encoder_resolution) * DEGREES_PER_REVOLUTION;

  // Apply offset
  degrees += encoder->offset_degrees;

  // Normalize to 0-360 range
  while (degrees >= DEGREES_PER_REVOLUTION)
    degrees -= DEGREES_PER_REVOLUTION;
  while (degrees < 0)
    degrees += DEGREES_PER_REVOLUTION;

  return degrees;
}

// Calculate absolute angle from accumulated steps
void encoder_update_absolute_angle(encoder_t *encoder) {
  if (!encoder)
    return;

  // Convert accumulated steps to degrees
  float total_degrees =
      ((float)encoder->accumulated_steps / encoder->encoder_resolution) *
      DEGREES_PER_REVOLUTION;

  // Apply gear ratio and inversion
  encoder->motor_angle_degrees = total_degrees;
  encoder->angle_degrees = total_degrees / encoder->gear_ratio;

  // Apply offset
  encoder->angle_degrees += encoder->offset_degrees;
}

// Enhanced encoder initialization
esp_err_t encoder_init(encoder_t *encoder) {
  if (!encoder || !encoder->i2c_master || !encoder->i2c_slave) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!encoder->i2c_master->bus_handle || !encoder->i2c_slave->dev_cfg ||
      !encoder->i2c_slave->dev_handle) {
    return ESP_ERR_INVALID_STATE;
  }

  // Set default values if not configured
  if (encoder->encoder_resolution == 0) {
    encoder->encoder_resolution = DEFAULT_ENCODER_RESOLUTION;
  }
  if (encoder->gear_ratio == 0.0f) {
    encoder->gear_ratio = 1.0f; // No gearing by default
  }

  ESP_LOGI(encoder->label, "Initialized with resolution: %d, gear ratio: %.2f",
           encoder->encoder_resolution, encoder->gear_ratio);

  // Add encoder device to the bus
  return i2c_master_bus_add_device(*encoder->i2c_master->bus_handle,
                                   encoder->i2c_slave->dev_cfg,
                                   encoder->i2c_slave->dev_handle);
}

// Your existing read function with minor improvements
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
      break;
    }

    ret = i2c_master_transmit_receive(*encoder->i2c_slave->dev_handle, &reg, 1,
                                      data, 2,
                                      encoder->i2c_master->timeout_ticks);
    if (ret != ESP_OK) {
      ESP_LOGE(encoder->label, "Read error: %s", esp_err_to_name(ret));
      break;
    }

    // Success - process the raw data
    uint16_t raw = ((data[0] << 8) | data[1]) & encoder->reg_angle_mask;

    // Apply inversion if needed (cleaner approach)
    if (encoder->is_inverted) {
      raw = encoder->reg_angle_mask - raw;
    }

    encoder->current_reading = raw;
    xSemaphoreGive(encoder->i2c_master->i2c_mutex);
    return raw;

  } while (0);

  // ---- I2C access failed ----
  xSemaphoreGive(encoder->i2c_master->i2c_mutex);
  return 0xFFFF;
}

// Utility functions for external access
float encoder_get_angle_degrees(encoder_t *encoder) {
  return encoder ? encoder->angle_degrees : 0.0f;
}

float encoder_get_motor_angle_degrees(encoder_t *encoder) {
  return encoder ? encoder->motor_angle_degrees : 0.0f;
}

int32_t encoder_get_accumulated_steps(encoder_t *encoder) {
  return encoder ? encoder->accumulated_steps : 0;
}

// Reset/calibration functions
esp_err_t encoder_zero_position(encoder_t *encoder) {
  if (!encoder)
    return ESP_ERR_INVALID_ARG;

  encoder->accumulated_steps = 0;
  encoder->angle_degrees = 0.0f;
  encoder->motor_angle_degrees = 0.0f;
  encoder->is_calibrated = true;

  ESP_LOGI(encoder->label, "Position zeroed");
  return ESP_OK;
}
