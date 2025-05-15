#include "scara_controller.h"

switch_t switch_1 = {
    .value = false,
    .gpio_port = GPIO_NUM_4,
};

gpio_config_t switch_1_io_conf = {
    /* .pin_bit_mask = (1ULL << switch_1->gpio_port), */
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE // No interrupts for now
};

mag_encoder encoder_1 = {
    .raw_val = 0,
    .offset = 0,
    .is_calibrated = false,
    .cal_switch = &switch_1,
};

motor_t motor_x = {
    .id = MOTOR_X,
    .gpio_dir = GPIOXDIR,
    .gpio_stp = GPIOXSTP,
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_0,
    .mcpwm_opr = MCPWM_OPR_A,
    .move_ms = 1000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .acel = 500,
};

motor_t motor_y = {
    .id = MOTOR_Y,
    .gpio_dir = GPIOYDIR,
    .gpio_stp = GPIOYSTP,
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_1,
    .mcpwm_opr = MCPWM_OPR_A,
    .move_ms = 1000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .acel = 500,
};

motor_t motor_z = {
    .id = MOTOR_Z,
    .gpio_dir = GPIOZDIR,
    .gpio_stp = GPIOZSTP,
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_2,
    .mcpwm_opr = MCPWM_OPR_A,
    .move_ms = 1500,
};

void init_scara() {

  init_motor_dir(&motor_x);
  init_motor_stp(&motor_x);

  init_motor_dir(&motor_y);
  init_motor_stp(&motor_y);

  init_motor_dir(&motor_z);
  init_motor_stp(&motor_z);
  ESP_LOGI("init_scara", "init_scara ended without errors!");

  xTaskCreate(accel_test_motor, "accel_motor_x_task", 2048, (void *)&motor_x,
              10, NULL);
  xTaskCreate(accel_test_motor, "accel_motor_y_task", 2048, (void *)&motor_y,
              10, NULL);
  xTaskCreate(accel_test_motor, "accel_motor_z_task", 2048, (void *)&motor_z,
              10, NULL);
  return;
}
void loop_scara() {
  while (true) {
    ESP_LOGI("loop_scara", "Changing freq to 800 hz");
    motor_x.target_freq_hz = 1000;
    motor_y.target_freq_hz = 1000;
    motor_z.target_freq_hz = 1000;
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    ESP_LOGI("loop_scara", "Changing freq to 200 hz");
    motor_x.target_freq_hz = 0;
    motor_y.target_freq_hz = 0;
    motor_z.target_freq_hz = 0;
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    /* vTaskDelay(1000 / portTICK_PERIOD_MS); */
  }
  return;
}
