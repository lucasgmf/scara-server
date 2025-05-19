#include "scara_tasks.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char *TAG = "scara_tasks";

#define TASK_UPDATE_MOTOR_PWM_PERIOD_MS 20

void task_update_motor_pwm(void *arg) {
  motor_t *motor_n = (motor_t *)arg;
  if (motor_n == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  TickType_t last_wake_time = xTaskGetTickCount();
  const float dt_sec = TASK_UPDATE_MOTOR_PWM_PERIOD_MS / 1000.0f;

  while (true) {
    /* ESP_LOGI(TAG, "Current freq: %.2f Hz, Target: %d Hz", */
    /*          motor_n->current_freq_hz, motor_n->target_freq_hz); */

    float diff = motor_n->target_freq_hz - motor_n->current_freq_hz;
    float step = motor_n->speed_hz * dt_sec;

    // Ramp frequency toward target
    if (fabsf(diff) < step) {
      motor_n->current_freq_hz = motor_n->target_freq_hz;
    } else {
      motor_n->current_freq_hz += (diff > 0) ? step : -step;
    }

    // Update step count estimate
    motor_n->step_count += (int)(motor_n->current_freq_hz * dt_sec);

    // Update PWM
    motor_delete_pwm(motor_n);
    if (motor_n->current_freq_hz > 0.0f) {
      motor_create_pwm(motor_n);
    }

    // Wait for next tick
    vTaskDelayUntil(&last_wake_time,
                    pdMS_TO_TICKS(TASK_UPDATE_MOTOR_PWM_PERIOD_MS));
  }
}

void encoder_task(void *param) {
  encoder_t *enc = (encoder_t *)param;
  while (1) {
    encoder_read_angle(enc);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  return;
}
