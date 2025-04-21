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

void motor_test(int gpiodir, int gpiostp) {
  gpio_reset_pin(gpiodir);
  gpio_reset_pin(gpiostp);
  gpio_set_direction(gpiodir, GPIO_MODE_OUTPUT);
  gpio_set_direction(gpiostp, GPIO_MODE_OUTPUT);

  while (1) {
    ESP_LOGI("Motor", "Changing motor direction to %d\n", 1);
    gpio_set_level(gpiodir, 1);
    for (int pulses = 0; pulses < 1000; pulses++) {
      gpio_set_level(gpiostp, 1);
      esp_rom_delay_us(500);
      gpio_set_level(gpiostp, 0);
      esp_rom_delay_us(500);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI("Motor", "Changing motor direction to %d\n", 0);
    gpio_set_level(gpiodir, 0);
    for (int pulses = 0; pulses < 1000; pulses++) {
      gpio_set_level(gpiostp, 1);
      esp_rom_delay_us(500);
      gpio_set_level(gpiostp, 0);
      esp_rom_delay_us(500);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void driver_calibration(int gpiodir, int gpiostp) {
  gpio_reset_pin(gpiodir);
  gpio_reset_pin(gpiostp);
  gpio_set_direction(gpiodir, GPIO_MODE_OUTPUT);
  gpio_set_direction(gpiostp, GPIO_MODE_OUTPUT);

  while (1) {
    ESP_LOGI("Motor", "You can now calibrate!\n", 0);
    gpio_set_level(gpiodir, 1);
    gpio_set_level(gpiostp, 1);
    vTaskDelay(10000000000 / portTICK_PERIOD_MS);
  }
}
