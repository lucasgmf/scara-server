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


void init_motor_stp(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op,
                    gpio_num_t gpio) {

  mcpwm_io_signals_t signal =
      (op == MCPWM_OPR_A) ? MCPWM0A + timer : MCPWM0B + timer;

  mcpwm_gpio_init(unit, signal, gpio);
}
