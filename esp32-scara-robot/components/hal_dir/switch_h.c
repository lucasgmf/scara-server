#include "switch_h.h"


static const char *TAG = "switch_h";

void switch_init(switch_t *switch_n) {
  if (switch_n == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    return;
  }
  gpio_config(switch_n->config);
}

void update_switch_val(switch_t *switch_n) {
  if (switch_n == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    return;
  }

  // inverting logic
  // now pressed means 1 and not_pressed means 0
  switch_n->is_pressed = !gpio_get_level(switch_n->gpio_pin);
  return;
}
