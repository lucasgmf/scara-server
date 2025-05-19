#include "motor.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char *TAG = "motor.c";

#define PWM_RESOLUTION_HZ 1000000

void motor_init_dir(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW(TAG, "Pointer is null in motor_init_dir\n");
    return;
  }
  gpio_reset_pin(motor_n->gpio_dir);
  gpio_set_direction(motor_n->gpio_dir, GPIO_MODE_OUTPUT);
  ESP_LOGW(TAG, "Successfully initializated motor %d", motor_n->id);

  return;
}

#define PWM_RESOLUTION_HZ 1000000 // 1 MHz
#define MCPWM_MIN_PERIOD_TICKS 5
#define PWM_FREQ_MIN 1.0f
#define PWM_FREQ_MAX ((float)PWM_RESOLUTION_HZ / (float)MCPWM_MIN_PERIOD_TICKS)
#define MCPWM_MAX_PERIOD_TICKS                                                 \
  60000 // Add a safety margin
        //
void motor_create_pwm(motor_t *motor) {
  if (!motor) {
    ESP_LOGE(TAG, "motor pointer is NULL");
    return;
  }

  motor_delete_pwm(motor);

  // Validate frequency
  if (motor->current_freq_hz < PWM_FREQ_MIN) {
    ESP_LOGW(TAG, "Skipping PWM creation: freq = %.2f is too low",
             motor->current_freq_hz);
    return;
  }

  // Clamp frequency to a safe range
  float freq = motor->current_freq_hz;
  if (freq > PWM_FREQ_MAX)
    freq = PWM_FREQ_MAX;
  if (freq < PWM_FREQ_MIN)
    freq = PWM_FREQ_MIN;

  uint32_t period_ticks = (uint32_t)(PWM_RESOLUTION_HZ / freq);
  if (period_ticks < MCPWM_MIN_PERIOD_TICKS)
    period_ticks = MCPWM_MIN_PERIOD_TICKS;
  if (period_ticks > MCPWM_MAX_PERIOD_TICKS)
    period_ticks = MCPWM_MAX_PERIOD_TICKS;

  uint32_t duty_ticks = period_ticks / 2;

  // Create timer
  mcpwm_timer_config_t timer_config = {
      .group_id = motor->mcpwm_unit,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = PWM_RESOLUTION_HZ,
      .period_ticks = period_ticks,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };

  esp_err_t err;
  err = mcpwm_new_timer(&timer_config, &motor->pwm_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(err));
    return;
  }

  // Create operator and connect timer
  mcpwm_operator_config_t operator_config = {.group_id = motor->mcpwm_unit};
  err = mcpwm_new_operator(&operator_config, &motor->pwm_oper);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM operator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_operator_connect_timer(motor->pwm_oper, motor->pwm_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to connect operator to timer: %s",
             esp_err_to_name(err));
    return;
  }

  // Create comparator
  mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez =
                                                     true};
  err = mcpwm_new_comparator(motor->pwm_oper, &comparator_config,
                             &motor->pwm_comparator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create comparator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_comparator_set_compare_value(motor->pwm_comparator, duty_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set compare value: %s", esp_err_to_name(err));
    return;
  }

  // Create generator
  mcpwm_generator_config_t generator_config = {.gen_gpio_num = motor->gpio_stp};
  err = mcpwm_new_generator(motor->pwm_oper, &generator_config,
                            &motor->pwm_generator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create generator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_timer_event(
      motor->pwm_generator,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY,
                                   MCPWM_GEN_ACTION_HIGH));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator high action: %s",
             esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_compare_event(
      motor->pwm_generator, MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, motor->pwm_comparator,
                                MCPWM_GEN_ACTION_LOW));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator low action: %s",
             esp_err_to_name(err));
    return;
  }

  // Enable and start timer
  err = mcpwm_timer_enable(motor->pwm_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_timer_start_stop(motor->pwm_timer, MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    return;
  }

  ESP_LOGI(TAG, "PWM created for motor %d at freq %.2f Hz (period %u ticks)",
           motor->id, freq, period_ticks);
}

void motor_delete_pwm(motor_t *motor) {
  if (!motor)
    return;

  if (motor->pwm_timer) {
    mcpwm_timer_disable(motor->pwm_timer); // Safe to disable before delete
    mcpwm_del_timer(motor->pwm_timer);
    motor->pwm_timer = NULL;
  }

  if (motor->pwm_generator) {
    mcpwm_del_generator(motor->pwm_generator);
    motor->pwm_generator = NULL;
  }

  if (motor->pwm_comparator) {
    mcpwm_del_comparator(motor->pwm_comparator);
    motor->pwm_comparator = NULL;
  }

  if (motor->pwm_oper) {
    mcpwm_del_operator(motor->pwm_oper);
    motor->pwm_oper = NULL;
  }
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
