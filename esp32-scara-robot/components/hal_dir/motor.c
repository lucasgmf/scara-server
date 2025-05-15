#include "motor.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

#define PWM_RESOLUTION_HZ 1000000

void motor_create_pwm(motor_t *motor) {
  if (motor->current_freq_hz <= 0) {
    ESP_LOGI("motor", "Skipping PWM creation: freq = 0");
    return; // Don't create PWM if frequency is 0
  }

  uint32_t period_ticks = PWM_RESOLUTION_HZ / (int)motor->current_freq_hz;
  uint32_t duty_ticks = period_ticks / 2;

  mcpwm_timer_config_t timer_config = {
      .group_id = motor->mcpwm_unit,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = PWM_RESOLUTION_HZ,
      .period_ticks = period_ticks,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor->pwm_timer));

  mcpwm_operator_config_t operator_config = {.group_id = motor->mcpwm_unit};
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motor->pwm_oper));
  ESP_ERROR_CHECK(
      mcpwm_operator_connect_timer(motor->pwm_oper, motor->pwm_timer));

  mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez =
                                                     true};
  ESP_ERROR_CHECK(mcpwm_new_comparator(motor->pwm_oper, &comparator_config,
                                       &motor->pwm_comparator));
  ESP_ERROR_CHECK(
      mcpwm_comparator_set_compare_value(motor->pwm_comparator, duty_ticks));

  mcpwm_generator_config_t generator_config = {.gen_gpio_num = motor->gpio_stp};
  ESP_ERROR_CHECK(mcpwm_new_generator(motor->pwm_oper, &generator_config,
                                      &motor->pwm_generator));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      motor->pwm_generator,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY,
                                   MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      motor->pwm_generator, MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, motor->pwm_comparator,
                                MCPWM_GEN_ACTION_LOW)));

  ESP_ERROR_CHECK(mcpwm_timer_enable(motor->pwm_timer));
  ESP_ERROR_CHECK(
      mcpwm_timer_start_stop(motor->pwm_timer, MCPWM_TIMER_START_NO_STOP));
}

void motor_delete_pwm(motor_t *motor) {
  if (motor->pwm_timer)
    mcpwm_timer_disable(motor->pwm_timer);
  if (motor->pwm_generator)
    mcpwm_del_generator(motor->pwm_generator);
  if (motor->pwm_comparator)
    mcpwm_del_comparator(motor->pwm_comparator);
  if (motor->pwm_oper)
    mcpwm_del_operator(motor->pwm_oper);
  if (motor->pwm_timer)
    mcpwm_del_timer(motor->pwm_timer);
  motor->pwm_timer = NULL;
  motor->pwm_oper = NULL;
  motor->pwm_comparator = NULL;
  motor->pwm_generator = NULL;
}

#define UPDATE_INTERVAL_MS 100 // timer period

void motor_update_timer_cb(void *arg) {
  motor_t *motor = (motor_t *)arg;

  int steps_this_period =
      (int)(motor->current_freq_hz * UPDATE_INTERVAL_MS / 1000.0f);

  // Accumulate steps
  motor->step_count += steps_this_period;

  float step = motor->speed_hz * 0.1f;
  float diff = motor->target_freq_hz - motor->current_freq_hz;

  if (fabs(diff) < step) {
    motor->current_freq_hz = motor->target_freq_hz;
    esp_timer_stop(motor->update_timer);
  } else {
    motor->current_freq_hz += (diff > 0 ? step : -step);
  }

  motor_delete_pwm(motor);

  if (motor->current_freq_hz > 0.0f) {
    motor_create_pwm(motor); // Only restart if freq > 0
  } else {
    ESP_LOGI("motor", "PWM stopped at 0 Hz");
  }
}

void motor_set_frequency(motor_t *motor, int target_freq_hz) {
  motor->target_freq_hz = target_freq_hz;

  if (motor->update_timer == NULL) {
    const esp_timer_create_args_t timer_args = {.callback =
                                                    &motor_update_timer_cb,
                                                .arg = motor,
                                                .name = "motor_freq_updater"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motor->update_timer));
  }

  ESP_ERROR_CHECK(
      esp_timer_start_periodic(motor->update_timer, 100000)); // 100ms
}
