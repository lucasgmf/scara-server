#include "scara_controller.h"

motor_t motor1 = {
    .id = 0,
    .gpio_stp = GPIOXSTP,
    .gpio_dir = GPIO_NUM_27, // optional
    .step_count = 0,
    .mcpwm_unit = 0,
    .mcpwm_timer = 0,
    .mcpwm_opr = 0,
    .move_ms = 0,
    .current_freq_hz = 100,
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
    .speed_hz = 1000,
};

void init_scara() {
  motor_create_pwm(&motor1);
  motor_create_pwm(&motor2);
  return;
}

void loop_scara() {
  while (1) {
    ESP_LOGI("main", "Changing freq to 1000 hz");
    motor_set_frequency(&motor1, 1000);
    motor_set_frequency(&motor2, 1000);
    vTaskDelay(2500 / portTICK_PERIOD_MS);

    ESP_LOGI("main", "Changing freq to 0 hz");
    motor_set_frequency(&motor1, 0);
    motor_set_frequency(&motor2, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    ESP_LOGI("main", "step_count: %d", motor1.step_count);
    ESP_LOGI("main", "step_count: %d", motor2.step_count);
  }

  /* vTaskDelay(1000 / portTICK_PERIOD_MS); */
  return;
}
