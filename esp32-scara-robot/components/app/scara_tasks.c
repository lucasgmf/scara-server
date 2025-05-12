#include "scara_tasks.h"

#define UPDATE_ENCODER_TASK_PERIOD_MS 10
#define MOVE_TEST_MOTOR_PERIOD_MS 10

void move_test_motor(void *arg) {
  motor_t *motor_n = (motor_t *)arg;
  if (motor_n == NULL) {
    ESP_LOGE("move_test_motor", "parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOVE_TEST_MOTOR_PERIOD_MS));
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
    reading = get_as5600_reading(0, 0); // WARN: Brokenn!
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
