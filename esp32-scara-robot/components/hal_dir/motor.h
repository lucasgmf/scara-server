#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
  int id;
  gpio_num_t gpio_stp;
  gpio_num_t gpio_dir;
  int step_count;
  int mcpwm_unit;
  int mcpwm_timer;
  int mcpwm_opr;
  int move_ms;
  float current_freq_hz;
  int target_freq_hz;
  float acel;
} motor_t;

void init_motor_dir(motor_t *motor_n);
void init_motor_stp(motor_t *motor_n);

#endif // MOTOR_H
