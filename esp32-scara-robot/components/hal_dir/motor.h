#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

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
  float speed_hz; // Hz per second
  mcpwm_timer_handle_t pwm_timer;
  mcpwm_oper_handle_t pwm_oper;
  mcpwm_cmpr_handle_t pwm_comparator;
  mcpwm_gen_handle_t pwm_generator;
  esp_timer_handle_t update_timer;
} motor_t;

void motor_init_dir(motor_t *motor_n);
void motor_create_pwm(motor_t *motor);
void motor_set_frequency(motor_t *motor, int target_freq_hz);
void motor_delete_pwm(motor_t *motor);

#endif // MOTOR_H
