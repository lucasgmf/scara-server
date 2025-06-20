#ifndef ENCODER_H
#define ENCODER_H

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"
#include "i2c_multiplexer.h"

#include "switch_h.h"

// Enhanced encoder structure with better degree calculation
typedef struct {
  const char *label;
  i2c_master_config_t *i2c_master;
  i2c_slave_bus_params *i2c_slave;
  i2c_slave_bus_params *i2c_tca;
  uint8_t tca_channel;
  uint8_t reg_angle_msb;
  uint16_t reg_angle_mask;

  // Legacy compatibility fields
  float initial_offset;
  bool reverse;          // legacy reverse flag
  float current_reading; // legacy current reading

  // Enhanced calibration and conversion parameters
  float offset_degrees; // offset in degrees
  bool is_inverted;
  bool is_calibrated;
  float gear_ratio; // motor gear ratio (output/input)

  // Current state
  int32_t accumulated_steps;
  float angle_degrees;       // current absolute angle in degrees
  float motor_angle_degrees; // motor shaft angle (before gearing)

  // Configuration
  uint16_t encoder_resolution; // e.g., 4096 for 12-bit

  // Optional switch and test parameters
  switch_t *switch_n;
  int test_offset;
} encoder_t;

float encoder_raw_to_degrees(encoder_t *encoder, uint16_t raw_value);
void encoder_update_absolute_angle(encoder_t *encoder);
esp_err_t encoder_init(encoder_t *encoder);
uint16_t encoder_read_angle(encoder_t *encoder);
float encoder_get_angle_degrees(encoder_t *encoder);
float encoder_get_motor_angle_degrees(encoder_t *encoder);
int32_t encoder_get_accumulated_steps(encoder_t *encoder);
esp_err_t encoder_zero_position(encoder_t *encoder);

#endif // ENCODER_H
