#include <stdio.h>
#include "motor.h"


void motor_setup(void) {
  gpio_reset_pin(BLINK_LED);
  gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);

  while (1) {
    gpio_set_level(BLINK_LED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(BLINK_LED, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
