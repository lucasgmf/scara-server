#include "scara_controller.h"

static const char *TAG = "scara_controller";

motor_t motor1 = {
    .id = 0,
    .gpio_stp = GPIOXSTP,
    .gpio_dir = GPIO_NUM_27, // optional
    .step_count = 0,
    .mcpwm_unit = 0,
    .mcpwm_timer = 0,
    .mcpwm_opr = 0,
    .move_ms = 0,
    .current_freq_hz = 200,
    .target_freq_hz = 0,
    .speed_hz = 1000,
};

motor_t motor2 = {
    .id = 1,
    .gpio_stp = GPIOYSTP,
    .gpio_dir = GPIO_NUM_33, // optional
    .step_count = 0,
    .mcpwm_unit = 1,
    .mcpwm_timer = 0,
    .mcpwm_opr = 0,
    .move_ms = 0,
    .current_freq_hz = 200,
    .target_freq_hz = 0,
    .speed_hz = 1500,
};

void init_scara() {
  motor_create_pwm(&motor1);

  xTaskCreatePinnedToCore(task_update_motor_pwm, "task_motor_pwm_1", 2048,
                          (void *)&motor1, 5, NULL, 0);

  ESP_LOGI(TAG, "Changing freq to 1000 hz");
  return;
}

void loop_scara() {
  while (1) {
    ESP_LOGI(TAG, "Changing freq to 1000 hz");
    motor1.target_freq_hz = 1000;
    vTaskDelay(2500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Changing freq to 0 hz");
    motor1.target_freq_hz = 0;
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "step_count: %d", motor1.step_count);
  }

  /* vTaskDelay(1000 / portTICK_PERIOD_MS); */
  return;
}
