#include "motor.h"

void led_test(void) {
  gpio_reset_pin(BLINK_LED);
  gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);

  while (1) {
    gpio_set_level(BLINK_LED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(BLINK_LED, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void led_test_2(void) {
  gpio_reset_pin(BLINK_LED);
  gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);

  int direction = 0;

  while (1) {
    direction = !direction;
    gpio_set_level(BLINK_LED, direction);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("led_test_2", "\ndirection: %d\nvalue: %d",
             gpio_get_level(BLINK_LED));
  }
}

void motor_test(void) {
  gpio_reset_pin(GPIOXDIR);
  gpio_reset_pin(GPIOXSTP);
  gpio_set_direction(GPIOXDIR, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIOXSTP, GPIO_MODE_OUTPUT);

  int direction = false;
  while (1) {
    direction = !direction;
    ESP_LOGI("Motor", "Changing motor direction to %d\n", direction);

    gpio_set_level(GPIOXDIR, direction);

    ESP_LOGI("Motor", "Starting cycles\n");
    for (int pulses = 0; pulses < 800; pulses++) {
      gpio_set_level(GPIOXSTP, direction);
      esp_rom_delay_us(500);
      gpio_set_level(GPIOXSTP, direction);
      esp_rom_delay_us(500);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
