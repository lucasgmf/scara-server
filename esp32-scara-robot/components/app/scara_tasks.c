#include "scara_tasks.h"
#include "esp_timer.h"
#include "math.h"

#define UPDATE_ENCODER_TASK_PERIOD_MS 10
#define MOVE_TEST_MOTOR_PERIOD_MS 10

void accel_test_motor(void *arg) {
  motor_t *motor = (motor_t *)arg;
  const float duty_cycle = 50.0;

  float last_freq = 0.0;
  int64_t last_time = esp_timer_get_time();

  while (1) {
    int64_t now = esp_timer_get_time();
    float dt_sec = (now - last_time) / 1e6;
    last_time = now;

    int target = motor->target_freq_hz;
    float current = motor->current_freq_hz;
    float accel = (motor->acel > 0) ? motor->acel : 1;

    float df = accel * dt_sec;

    if (current < target) {
      current += df;
      if (current > target)
        current = target;
    } else if (current > target) {
      current -= df;
      if (current < target)
        current = target;
    }

    motor->current_freq_hz = current;

    /* ESP_LOGI("accel_test_motor", */
    /*          "Current freq: %.2f | Target freq: %d | dt_sec: %.6f | df: %.6f", */
    /*          current, target, dt_sec, df); */

    if ((int)current == 0) {
      mcpwm_stop(motor->mcpwm_unit, motor->mcpwm_timer);
      mcpwm_set_signal_low(motor->mcpwm_unit, motor->mcpwm_timer,
                           motor->mcpwm_opr);
      last_freq = 0.0;
    } else {
      bool should_reinit = false;

      if (last_freq == 0.0 && current >= 25.0) {
        should_reinit = true;
      } else if (fabs(current - last_freq) > 25.0) {
        should_reinit = true;
      }

      if (should_reinit) {
        mcpwm_config_t config = {
            .frequency = (int)current,
            .cmpr_a = duty_cycle,
            .cmpr_b = duty_cycle,
            .counter_mode = MCPWM_UP_COUNTER,
            .duty_mode = MCPWM_DUTY_MODE_0,
        };
        mcpwm_init(motor->mcpwm_unit, motor->mcpwm_timer, &config);
        last_freq = current;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void update_encoder_val_task(void *arg) {
  mag_encoder *encoder_n = (mag_encoder *)arg;
  if (encoder_n == NULL) {
    ESP_LOGE("update_encoder_val_task", "parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }
  TickType_t last_wake_time = xTaskGetTickCount();

  uint16_t reading;
  int delta;
  static int last_val = -1;

  while (true) {
    reading = get_as5600_reading(encoder_n, 0x0C); // WARN: Brokenn!
    check_encoder_cal(encoder_n, reading);

    if (last_val == -1) {
      last_val = reading;
      return;
    }

    delta = reading - last_val;
    if (delta > MAX_ENCODER_VAL / 2) {
      delta -= MAX_ENCODER_VAL;
    } else if (delta < -MAX_ENCODER_VAL / 2) {
      delta += MAX_ENCODER_VAL;
    }

    encoder_n->raw_val += delta;
    last_val = reading;
    vTaskDelayUntil(&last_wake_time,
                    pdMS_TO_TICKS(UPDATE_ENCODER_TASK_PERIOD_MS));
  }
  return;
}
