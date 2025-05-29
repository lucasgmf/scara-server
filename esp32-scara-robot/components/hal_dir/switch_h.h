#ifndef SWITCH_H_H
#define SWITCH_H_H

#include "driver/gpio.h"
#include "esp_log.h"

typedef struct {
  gpio_config_t *config;
  gpio_num_t gpio_pin;
  bool is_pressed;
} switch_t;

void switch_init(switch_t *switch_n);
void update_switch_val(switch_t *switch_n);

#endif // SWITCH_H_H
