#ifndef SWITCH_H_H
#define SWITCH_H_H

#include "driver/gpio.h"

typedef struct {
  gpio_num_t gpio_port;
  bool value;
} switch_t;

void init_switch(switch_t *switch_n, gpio_config_t *io_conf);
void update_switch_val(switch_t *switch_n);

#endif // SWITCH_H_H
