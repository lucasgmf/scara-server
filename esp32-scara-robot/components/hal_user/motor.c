#include "motor.h"

motor_t motor_x = {
    .id = MOTOR_X,
    .gpio_dir = GPIOXDIR,
    .gpio_stp = GPIOXSTP,
    .delay_us = 600,
};

motor_t motor_y = {
    .id = MOTOR_Y,
    .gpio_dir = GPIOYDIR,
    .gpio_stp = GPIOYSTP,
    .delay_us = 600,
};

motor_t motor_z = {
    .id = MOTOR_Z,
    .gpio_dir = GPIOZDIR,
    .gpio_stp = GPIOZSTP,
    .delay_us = 500,
};

/* motor_t motor_a = { */
/*     .id = MOTOR_A, */
/*     .gpio_dir = GPIOADIR, */
/*     .gpio_stp = GPIOASTP, */
/*     .delay_us = 500, */
/* }; */

motor_t *motor_list[] = {&motor_x, &motor_y, &motor_z};

motor_array_t motor_array = {
    .motors = motor_list,
    .count = 3,
};

// functions ----------------------------------------------------

void motor_init(motor_t *motor_n) {
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

void motor_init_all() {
  for (size_t i = 0; i < motor_array.count; i++) {
    motor_init(motor_array.motors[i]);
  }
  ESP_LOGI("motor_init_all", "All motors have been initializated");
}

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
