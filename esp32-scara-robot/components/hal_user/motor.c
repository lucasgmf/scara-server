#include "motor.h"

void motor_initialization(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("motor_initialization", "Pointer is null\n");
    return;
  }
  int gpiodir = motor_n->gpio_dir;
  int gpiostp = motor_n->gpio_stp;

  gpio_reset_pin(gpiodir);
  gpio_reset_pin(gpiostp);
  gpio_set_direction(gpiodir, GPIO_MODE_OUTPUT);
  gpio_set_direction(gpiostp, GPIO_MODE_OUTPUT);

  ESP_LOGW("motor_initialization", "Successfully initializated motor %d",
           motor_n->id);
  return;
}

void motor_test(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("motor_test", "Pointer is null\n");
    return;
  }
  int gpiodir = motor_n->gpio_dir;
  int gpiostp = motor_n->gpio_stp;

  gpio_reset_pin(gpiodir);
  gpio_reset_pin(gpiostp);
  gpio_set_direction(gpiodir, GPIO_MODE_OUTPUT);
  gpio_set_direction(gpiostp, GPIO_MODE_OUTPUT);

  while (1) {
    gpio_set_level(gpiodir, 1);
    for (int pulses = 0; pulses < 1000; pulses++) {
      gpio_set_level(gpiostp, 1);
      esp_rom_delay_us(500);
      gpio_set_level(gpiostp, 0);
      esp_rom_delay_us(500);
    }

    gpio_set_level(gpiodir, 0);
    for (int pulses = 0; pulses < 1000; pulses++) {
      gpio_set_level(gpiostp, 1);
      esp_rom_delay_us(500);
      gpio_set_level(gpiostp, 0);
      esp_rom_delay_us(500);
    }

    gpio_set_level(gpiodir, 1);
    for (int pulses = 0; pulses < 1000; pulses++) {
      gpio_set_level(gpiostp, 1);
      esp_rom_delay_us(600);
      gpio_set_level(gpiostp, 0);
      esp_rom_delay_us(600);
    }

    gpio_set_level(gpiodir, 0);
    for (int pulses = 0; pulses < 1000; pulses++) {
      gpio_set_level(gpiostp, 1);
      esp_rom_delay_us(700);
      gpio_set_level(gpiostp, 0);
      esp_rom_delay_us(700);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void driver_calibration(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("driver_calibration", "Pointer is null\n");
    return;
  }

  int gpiodir = motor_n->gpio_dir;
  int gpiostp = motor_n->gpio_stp;

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

void motor_update(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW("motor_update", "Null pointer at motor_update\n");
    return;
  }
  motor_move(motor_n, 1, 500, motor_n->delay_us);
  /* vTaskDelay(300 / portTICK_PERIOD_MS); */
  motor_move(motor_n, 0, 500, motor_n->delay_us);
  return;
}

void motor_move(motor_t *motor_n, bool direction, int iteractions,
                int delay_us) {
  if (motor_n == NULL) {
    ESP_LOGW("motor_move", "Null pointer at motor_update\n");
    return;
  }

  if (gpio_get_level(motor_n->gpio_dir != direction)) {
    gpio_set_level(motor_n->gpio_dir, direction);
  }
  for (int i = 0; i < iteractions; i++) {
    gpio_set_level(motor_n->gpio_stp, 1);
    esp_rom_delay_us(delay_us);
    gpio_set_level(motor_n->gpio_stp, 0);
    esp_rom_delay_us(delay_us);
  }
  return;
}
