#include "motor.h"

void init_motor(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("motor_initialization", "Pointer is null\n");
    return;
  }

  gpio_reset_pin(motor_n->gpio_dir);
  gpio_reset_pin(motor_n->gpio_stp);
  gpio_set_direction(motor_n->gpio_dir, GPIO_MODE_OUTPUT);
  gpio_set_direction(motor_n->gpio_stp, GPIO_MODE_OUTPUT);

  ESP_LOGW("motor_initialization", "Successfully initializated motor %d",
           motor_n->id);
  return;
}

