#include "scara_tasks.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char *TAG = "scara_tasks";


#define MAX_ENCODER_VAL 4096 // 12-bit encoder
#define HALF_ENCODER_VAL (MAX_ENCODER_VAL / 2)

void encoder_task(void *arg) {
  encoder_t *encoder = (encoder_t *)arg;
  if (encoder == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  // Configuration validation
  uint16_t max_encoder_val = encoder->encoder_resolution;
  int16_t half_encoder_val = max_encoder_val / 2;

  ESP_LOGI(encoder->label,
           "Starting encoder task - resolution: %d, gear ratio: %.2f",
           max_encoder_val, encoder->gear_ratio);

  // First read to establish baseline
  uint16_t last_angle = encoder_read_angle(encoder);
  if (last_angle == 0xFFFF) {
    ESP_LOGE(encoder->label, "Failed initial encoder read");
    vTaskDelete(NULL);
    return;
  }

  uint16_t current_angle;
  int16_t delta;
  uint32_t error_count = 0;

  while (1) {
    current_angle = encoder_read_angle(encoder);

    if (current_angle == 0xFFFF) {
      error_count++;
      ESP_LOGW(encoder->label, "Failed to read angle (error count: %lu)",
               error_count);

      // If too many consecutive errors, consider resetting or taking action
      if (error_count > 10) {
        ESP_LOGE(encoder->label,
                 "Too many consecutive read errors, continuing...");
        error_count = 0; // Reset counter
      }

      vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay on error
      continue;
    }

    error_count = 0; // Reset error counter on successful read

    // Calculate delta with wraparound handling
    delta = (int16_t)current_angle - (int16_t)last_angle;

    // Handle wraparound (shortest path)
    if (delta > half_encoder_val) {
      delta -= max_encoder_val;
    } else if (delta < -half_encoder_val) {
      delta += max_encoder_val;
    }

    // Update accumulated steps
    encoder->accumulated_steps += delta;

    // Update angle calculations
    encoder_update_absolute_angle(encoder);

    // Optional: Log current position periodically
    /* static uint32_t log_counter = 0; */
    /* if (++log_counter % 100 == 0) { // Log every 5 seconds at 50ms intervals
     */
    /*   ESP_LOGI(encoder->label, */
    /*            "Raw: %d, Steps: %ld, Motor: %.2f°, Output: %.2f°", */
    /*            current_angle, encoder->accumulated_steps, */
    /*            encoder->motor_angle_degrees, encoder->angle_degrees); */
    /* } */
    /**/
    last_angle = current_angle;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
/* ESP_LOGI(encoder->label, "Raw angle: %u", current_angle); */
/* ESP_LOGI(encoder->label, */
/*          "Angle: %u | Delta: %d | Position: %ld | angle_deg: %.2f | " */
/*          "angle_rad :%.2f", */
/*          current_angle, delta, encoder->accumulated_steps, */
/*          encoder->accumulated_steps * 360.0 / 4096 / encoder->gear_ratio,
 */
/*          encoder->accumulated_steps * 2 * M_PI / encoder->gear_ratio / */
/*              4096); */

#define MOTOR_CONTROL_TASK_PERIOD_MS 20

void motor_control_task(void *arg) {
  motor_t *motor = (motor_t *)arg;
  if (motor == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  float error = 0;
  float output = 0;
  float local_target_freq_hz = 0;

  while (1) {

    if (!motor->control_vars->ref_encoder->is_calibrated) {
      /* ESP_LOGI("motor_control_task", "encoder is not calibrated"); */
      vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
      continue;
    }

    error = motor->control_vars->encoder_target_pos -
            motor->control_vars->ref_encoder->accumulated_steps;

    const float DEAD_BAND_THRESHOLD = 2.0f;            // WARN: define this
    float dt = MOTOR_CONTROL_TASK_PERIOD_MS / 1000.0f; // Time in seconds

    // Deadband: stop motor and prevent integral windup
    if (fabsf(error) < DEAD_BAND_THRESHOLD) {
      motor->control_vars->pid->integral = 0.0f;
      motor->pwm_vars->target_freq_hz = 0.0f;
      motor_set_target_frequency(motor, 0.0f);
      vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
      continue;
    }

    // PID calculations
    motor->control_vars->pid->integral += error * dt;

    // Optional: limit the integral term to prevent windup
    const float INTEGRAL_LIMIT = 2000.0f * 4;
    if (motor->control_vars->pid->integral > INTEGRAL_LIMIT)
      motor->control_vars->pid->integral = INTEGRAL_LIMIT;
    else if (motor->control_vars->pid->integral < -INTEGRAL_LIMIT)
      motor->control_vars->pid->integral = -INTEGRAL_LIMIT;

    float derivative = (error - motor->control_vars->pid->prev_error) / dt;
    motor->control_vars->pid->prev_error = error;

    output = motor->control_vars->pid->Kp * error +
             motor->control_vars->pid->Ki * motor->control_vars->pid->integral +
             motor->control_vars->pid->Kd * derivative;

    if (false) {
      ESP_LOGI("PID",
               "error: %.2f - output: %.2f | "
               "Kp*error = %.2f, Ki* integral = %.2f, Kd * derivative = %.2f",

               error, output, motor->control_vars->pid->Kp * error,
               motor->control_vars->pid->Ki *
                   motor->control_vars->pid->integral,
               motor->control_vars->pid->Kd * derivative);
    }

    // Clamp output to allowed frequency
    if (output > motor->pwm_vars->max_freq)
      output = motor->pwm_vars->max_freq;
    if (output < -motor->pwm_vars->max_freq)
      output = -motor->pwm_vars->max_freq;

    // WARN: maybe this is target_freq_hz??
    /* loop->output_freq_hz = fabsf(output); */

    /* ESP_LOGI("motor_control_task", "saving target freq to %f",
     * fabs(output));
     */
    local_target_freq_hz = fabsf(output);

    // Determine direction
    bool reverse = output < 0;

    // TODO: If direction is the same as previous, do not set level again...
    /* ESP_LOGW("debug", "setting dir to %d", */
    /*          motor->pwm_vars->dir_is_reversed ? !reverse : !reverse); */

    gpio_set_level(motor->gpio_dir,
                   motor->pwm_vars->dir_is_reversed ? !reverse : reverse);

    // Apply new frequency to motor
    motor_set_target_frequency(motor, local_target_freq_hz);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
  }
}

void switch_task(void *arg) {
  switch_t *switch_n = (switch_t *)arg;
  if (switch_n == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    update_switch_val(switch_n);
    /* ESP_LOGI("switch_task", "value of switch_n is %d", switch_n->is_pressed);
     */
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void hx711_task(void *arg) {
  hx711_t *hx711_n = (hx711_t *)arg;
  if (hx711_n == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    hx711_read_raw(hx711_n);
    ESP_LOGI("hx711_task", "value of hx711 with label %s is %d", hx711_n->label,
             hx711_n->raw_read);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void hx711_avg_task(void *arg) {
  hx711_t *hx711 = (hx711_t *)arg;
  if (hx711 == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    int64_t sum = 0;
    int num_samples = 10;

    for (int s = 0; s < num_samples; s++) {
      // Wait until HX711 is ready
      while (gpio_get_level(hx711->data_pin) == 1) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      // Read 24 bits
      uint32_t data = 0;
      for (int i = 0; i < 24; i++) {
        gpio_set_level(hx711->clock_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        data = (data << 1) | gpio_get_level(hx711->data_pin);
        gpio_set_level(hx711->clock_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      // Set gain (1:128, 2:64, 3:32)
      for (int i = 0; i < hx711->gain; i++) {
        gpio_set_level(hx711->clock_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(hx711->clock_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      // Sign-extend 24-bit to 32-bit
      if (data & 0x800000) {
        data |= 0xFF000000;
      }

      int32_t signed_data = (int32_t)data;
      sum += signed_data;

      vTaskDelay(pdMS_TO_TICKS(25));
    }

    int32_t average = sum / num_samples;
    ESP_LOGI("average", "Average HX711 reading: %ld\n", average);
  }
}
