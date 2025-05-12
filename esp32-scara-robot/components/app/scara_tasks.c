#include "scara_tasks.h"

#define UPDATE_ENCODER_TASK_PERIOD_MS 10
#define MOVE_TEST_MOTOR_PERIOD_MS 10

void move_test_motor_x(void *arg) {
  motor_t *motor_n = (motor_t *)arg;
  if (motor_n == NULL) {
    ESP_LOGE("move_test_motor", "parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    apply_motor_pwm(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0, 500);

    ESP_LOGI("loop_scara", "Changing direction of motor %d to 1", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction of motor %d to 0", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI("loop_scara", "Changing direction of motor %d to 1", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction of motor %d to 0", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOVE_TEST_MOTOR_PERIOD_MS));
  }
}

void move_test_motor_y(void *arg) {
  motor_t *motor_n = (motor_t *)arg;
  if (motor_n == NULL) {
    ESP_LOGE("move_test_motor", "parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    apply_motor_pwm(MCPWM_UNIT_0, MCPWM_TIMER_1, 50.0, 500);

    ESP_LOGI("loop_scara", "Changing direction of motor %d to 1", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction of motor %d to 0", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI("loop_scara", "Changing direction of motor %d to 1", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction of motor %d to 0", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOVE_TEST_MOTOR_PERIOD_MS));
  }
}

void move_test_motor_z(void *arg) {
  motor_t *motor_n = (motor_t *)arg;
  if (motor_n == NULL) {
    ESP_LOGE("move_test_motor", "parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    apply_motor_pwm(MCPWM_UNIT_0, MCPWM_TIMER_2, 50.0, 700);
    ESP_LOGI("loop_scara", "Changing direction of motor %d to 1", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction of motor %d to 0", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI("loop_scara", "Changing direction of motor %d to 1", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction of motor %d to 0", motor_n->id);
    gpio_set_level(motor_n->gpio_dir, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

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
