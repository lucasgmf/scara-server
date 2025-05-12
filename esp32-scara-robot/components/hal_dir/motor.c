#include "motor.h"

void init_motor_dir(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("motor_initialization", "Pointer is null\n");
    return;
  }

  gpio_reset_pin(motor_n->gpio_dir);
  gpio_set_direction(motor_n->gpio_dir, GPIO_MODE_OUTPUT);

  ESP_LOGW("motor_initialization", "Successfully initializated motor %d",
           motor_n->id);
  return;
}

void init_motor_stp(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("init_motor_stp", "Pointer is null\n");
    return;
  }
  mcpwm_io_signals_t signal = (motor_n->mcpwm_opr == MCPWM_OPR_A)
                                  ? MCPWM0A + motor_n->mcpwm_timer
                                  : MCPWM0B + motor_n->mcpwm_timer;

  mcpwm_gpio_init(motor_n->mcpwm_unit, signal, motor_n->gpio_stp);
  return;
}
