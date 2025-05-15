#include "motor_d.h"

void motor_move(motor_t *motor_n, bool direction, int iteractions,
                int delay_us) {
  if (motor_n == NULL) {
    ESP_LOGW("motor_move", "Null pointer at motor_update\n");
    return;
  }

  if (gpio_get_level(motor_n->gpio_dir != direction)) {
    gpio_set_level(motor_n->gpio_dir, direction);
  }
  for (int i = 0; i < iteractions; i++) {
    gpio_set_level(motor_n->gpio_stp, 1);
    esp_rom_delay_us(delay_us);
    gpio_set_level(motor_n->gpio_stp, 0);
    esp_rom_delay_us(delay_us);
  }
  return;
}

void check_motor_freq(motor_t *motor, int frequency_hz, float duty_cycle) {
  if (frequency_hz > 0) {
    mcpwm_config_t config = {
        .frequency = frequency_hz,
        .cmpr_a = duty_cycle,
        .cmpr_b = duty_cycle,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(motor->mcpwm_unit, motor->mcpwm_timer, &config);
  } else {
    mcpwm_stop(motor->mcpwm_unit, motor->mcpwm_timer);
    mcpwm_set_signal_low(motor->mcpwm_unit, motor->mcpwm_timer,
                         motor->mcpwm_opr);
  }
}
