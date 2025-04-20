#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLINK_LED 2

void app_main(void) {
  gpio_reset_pin(BLINK_LED);
  gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);

  while(1){
      gpio_set_level(BLINK_LED, 1);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      gpio_set_level(BLINK_LED, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
