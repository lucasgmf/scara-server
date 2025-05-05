#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


typedef struct {
  int id;
  gpio_num_t gpio_stp;
  gpio_num_t gpio_dir;
  int step_count;
  int target_steps;
  int speed; // TODO: does this makes sense?
  int period_ms;
  int delay_us;
} motor_t;


void init_motor(motor_t *motor_n);

#endif // MOTOR_H
