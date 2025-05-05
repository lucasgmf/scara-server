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
