#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "math.h"
#include "switch_h.h"

#define UPDATE_INTERVAL_MS 20
typedef struct {
  int group_unit;
  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t operator;
  mcpwm_cmpr_handle_t comparator;
  mcpwm_gen_handle_t generator;
  esp_timer_handle_t esp_timer_handle;
  int mcpwm_min_period_ticks;
  int mcpwm_max_period_ticks;
  int pwm_resolution_hz;
} motor_mcpwm_vars;

typedef struct {
  int step_count;
  float max_freq;
  float min_freq;
  float max_accel;
  float current_freq_hz;
  float target_freq_hz;
  float velocity_hz_per_s;

  bool dir_is_reversed;
} motor_pwm_vars_t;

#include "encoder.h"
#include "pid.h"

typedef struct {
  encoder_t *ref_encoder;
  float encoder_target_pos;

  bool enable_pid;
  pid_controller_t *pid;
  switch_t *ref_switch;
} motor_control_vars;

typedef struct {
  const char *label;
  int id;
  gpio_num_t gpio_stp;
  gpio_num_t gpio_dir;
  motor_mcpwm_vars *mcpwm_vars;
  motor_pwm_vars_t *pwm_vars;
  motor_control_vars *control_vars;
  bool is_inverted;
} motor_t;

void motor_init_dir(motor_t *motor_n);
void motor_delete_pwm(motor_t *motor);
void motor_create_pwm(motor_t *motor);
esp_err_t motor_init_timer(motor_t *motor);
esp_err_t motor_set_frequency_immediate(motor_t *motor, float freq_hz);
void motor_delete_timer(motor_t *motor);
esp_err_t motor_set_target_frequency(motor_t *motor, float target_freq_hz);
esp_err_t motor_create_pwm_with_frequency(motor_t *motor, float freq_hz);



#endif // MOTOR_H
