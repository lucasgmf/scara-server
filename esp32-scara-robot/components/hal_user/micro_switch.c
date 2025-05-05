#include "micro_switch.h"
#include "driver/gpio.h"

void switch_init(switch_t *switch_) {
  gpio_reset_pin(switch_->gpio_port);
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << switch_->gpio_port),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE // No interrupts for now
  };
  gpio_config(&io_conf);
  return;
}

void update_switch_val(switch_t *switch_) {
  switch_->value = gpio_get_level(switch_->gpio_port);
  return;
}
