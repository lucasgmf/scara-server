#include "micro_switch.h"
#include "driver/gpio.h"

/* void button_init() { */
/*   gpio_config_t io_conf = { */
/*       .pin_bit_mask = (1ULL << BUTTON_GPIO), */
/*       .mode = GPIO_MODE_INPUT, */
/*       .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up */
/*       .pull_down_en = GPIO_PULLDOWN_DISABLE, */
/*       .intr_type = GPIO_INTR_DISABLE // No interrupts for now */
/*   }; */
/*   gpio_config(&io_conf); */
/* } */

void button_init_func() {
  gpio_reset_pin(GPIO_NUM_4);
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << GPIO_NUM_4),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE // No interrupts for now
  };
  gpio_config(&io_conf);
}
