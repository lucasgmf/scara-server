#ifndef MICRO_SWITCH_H
#define MICRO_SWITCH_H

#include "driver/gpio.h"

typedef struct {
  gpio_num_t gpio_port;
  bool value;
} switch_t;

void switch_init(switch_t *switch_);
void update_switch_val(switch_t *switch_);

#endif // MICRO_SWITCH_H
