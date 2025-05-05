#include "switch_h.h"
#include "driver/gpio.h"

// TODO: add null pointer checks...
void init_switch(switch_t *switch_n, gpio_config_t *io_conf) {
  io_conf->pin_bit_mask = (1ULL << switch_n->gpio_port);
  gpio_reset_pin(switch_n->gpio_port);
  gpio_config(io_conf);
  return;
}

void update_switch_val(switch_t *switch_n) {
  switch_n->value = gpio_get_level(switch_n->gpio_port);
  return;
}
