#include "motor_driver.h"

void driver_calib(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("driver_calibration", "Pointer is null\n");
    return;
  }

  gpio_reset_pin(motor_n->gpio_dir);
  gpio_reset_pin(motor_n->gpio_stp);
  gpio_set_direction(motor_n->gpio_dir, GPIO_MODE_OUTPUT);
  gpio_set_direction(motor_n->gpio_stp, GPIO_MODE_OUTPUT);

  ESP_LOGI("Motor", "You can now calibrate!\n", 0);
  gpio_set_level(motor_n->gpio_dir, true);
  gpio_set_level(motor_n->gpio_stp, true);
  vTaskDelay(10000000000 / portTICK_PERIOD_MS);
  return;
}
