#include "motor.h"

static const char *TAG = "motor.c";

void motor_init_dir(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGI(TAG, "Pointer is null in motor_init_dir\n");
    return;
  }
  gpio_reset_pin(motor_n->gpio_dir);
  gpio_set_direction(motor_n->gpio_dir, GPIO_MODE_OUTPUT);
  ESP_LOGI(TAG, "Successfully initializated dir from motor %d", motor_n->id);

  return;
}

void motor_delete_pwm(motor_t *motor) {
  if (motor == NULL) {
    ESP_LOGW("motor_delete_pwm", "Pointer is null in motor_delete_pwm");
    return;
  }

  if (motor->mcpwm_vars->timer) {
    mcpwm_timer_disable(motor->mcpwm_vars->timer);
    mcpwm_del_timer(motor->mcpwm_vars->timer);
    motor->mcpwm_vars->timer = NULL;
  }

  if (motor->mcpwm_vars->generator) {
    mcpwm_del_generator(motor->mcpwm_vars->generator);
    motor->mcpwm_vars->generator = NULL;
  }

  if (motor->mcpwm_vars->comparator) {
    mcpwm_del_comparator(motor->mcpwm_vars->comparator);
    motor->mcpwm_vars->comparator = NULL;
  }

  if (motor->mcpwm_vars->operator) {
    mcpwm_del_operator(motor->mcpwm_vars->operator);
    motor->mcpwm_vars->operator= NULL;
  }
}

void motor_create_pwm(motor_t *motor) {
  if (motor == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  motor_delete_pwm(motor);
  if (motor->pwm_vars->current_freq_hz < motor->pwm_vars->min_freq) {
    ESP_LOGW(TAG, "Skipping PWM creation: freq = %.2f is too low",
             motor->pwm_vars->current_freq_hz);
    return;
  }

  // Clamp frequency to a safe range
  float freq = motor->pwm_vars->current_freq_hz;
  if (freq > motor->pwm_vars->max_freq)
    freq = motor->pwm_vars->max_freq;
  if (freq < motor->pwm_vars->min_freq)
    freq = motor->pwm_vars->min_freq;

  uint32_t period_ticks =
      (uint32_t)(motor->mcpwm_vars->pwm_resolution_hz / freq);
  if (period_ticks < motor->mcpwm_vars->mcpwm_min_period_ticks)
    period_ticks = motor->mcpwm_vars->mcpwm_min_period_ticks;
  if (period_ticks > motor->mcpwm_vars->mcpwm_max_period_ticks)
    period_ticks = motor->mcpwm_vars->mcpwm_max_period_ticks;

  uint32_t duty_ticks = period_ticks / 2;

  // Create timer
  mcpwm_timer_config_t timer_config = {
      .group_id = motor->mcpwm_vars->group_unit,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = motor->mcpwm_vars->pwm_resolution_hz,
      .period_ticks = period_ticks,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };

  esp_err_t err;
  err = mcpwm_new_timer(&timer_config, &motor->mcpwm_vars->timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(err));
    return;
  }

  // Create operator and connect timer
  mcpwm_operator_config_t operator_config = {
      .group_id = motor->mcpwm_vars->group_unit,
  };

  err = mcpwm_new_operator(&operator_config, &motor->mcpwm_vars->operator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM operator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_operator_connect_timer(motor->mcpwm_vars->operator,
                                     motor->mcpwm_vars->timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to connect operator to timer: %s",
             esp_err_to_name(err));
    return;
  }

  // Create comparator
  mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez =
                                                     true};
  err = mcpwm_new_comparator(motor->mcpwm_vars->operator, & comparator_config,
                             &motor->mcpwm_vars->comparator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create comparator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_comparator_set_compare_value(motor->mcpwm_vars->comparator,
                                           duty_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set compare value: %s", esp_err_to_name(err));
    return;
  }

  // Create generator
  mcpwm_generator_config_t generator_config = {.gen_gpio_num = motor->gpio_stp};
  err = mcpwm_new_generator(motor->mcpwm_vars->operator, & generator_config,
                            &motor->mcpwm_vars->generator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create generator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_timer_event(
      motor->mcpwm_vars->generator,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY,
                                   MCPWM_GEN_ACTION_HIGH));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator high action: %s",
             esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_compare_event(
      motor->mcpwm_vars->generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     motor->mcpwm_vars->comparator,
                                     MCPWM_GEN_ACTION_LOW));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator low action: %s",
             esp_err_to_name(err));
    return;
  }

  // Enable and start timer
  err = mcpwm_timer_enable(motor->mcpwm_vars->timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_timer_start_stop(motor->mcpwm_vars->timer,
                               MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    return;
  }

  /* ESP_LOGI(TAG, "PWM created for motor %d at freq %.2f Hz (period %u ticks)",
   */
  /*          motor->id, freq, period_ticks); */
}

void motor_set_frequency(motor_t *motor, float target_freq_hz) {
  if (motor->pwm_vars->target_freq_hz == target_freq_hz) {
    return;
  }

  motor->pwm_vars->target_freq_hz = target_freq_hz;

  if (motor->mcpwm_vars->esp_timer_handle == NULL) {
    const esp_timer_create_args_t timer_args = {.callback =
                                                    &motor_update_timer_cb,
                                                .arg = motor,
                                                .name = "motor_freq_updater"};
    ESP_ERROR_CHECK(
        esp_timer_create(&timer_args, &motor->mcpwm_vars->esp_timer_handle));
  }

  if (!esp_timer_is_active(motor->mcpwm_vars->esp_timer_handle)) {
    ESP_ERROR_CHECK(esp_timer_start_periodic(
        motor->mcpwm_vars->esp_timer_handle, UPDATE_INTERVAL_MS * 1000));
  }
  ESP_LOGW("motor_set_frequency", "just set frequency to %f", target_freq_hz);
}

void motor_update_timer_cb(void *arg) {
  motor_t *motor = (motor_t *)arg;

  const float dt = UPDATE_INTERVAL_MS / 1000.0f; // seconds
  float *current_freq = &motor->pwm_vars->current_freq_hz;
  float *target_freq = &motor->pwm_vars->target_freq_hz;
  float *velocity =
      &motor->pwm_vars
           ->velocity_hz_per_s; // ← you need to add this to your struct
  float max_accel = motor->pwm_vars->max_accel; // in Hz/sec

  // Frequency error
  float freq_error = *target_freq - *current_freq;

  // Compute desired velocity
  float desired_velocity = freq_error / dt;

  // Limit the velocity change (acceleration)
  float accel_step = max_accel * dt;

  if (desired_velocity > *velocity + accel_step) {
    *velocity += accel_step;
  } else if (desired_velocity < *velocity - accel_step) {
    *velocity -= accel_step;
  } else {
    *velocity = desired_velocity;
  }

  // Apply velocity to frequency (integrate)
  *current_freq += (*velocity * dt);

  // Clamp overshoot
  if ((freq_error > 0 && *current_freq > *target_freq) ||
      (freq_error < 0 && *current_freq < *target_freq)) {
    *current_freq = *target_freq;
    *velocity = 0;
  }

  // Stop timer if target is reached
  if (fabs(*target_freq - *current_freq) < 0.5f && fabs(*velocity) < 1.0f) {
    *current_freq = *target_freq;
    *velocity = 0;
    esp_timer_stop(motor->mcpwm_vars->esp_timer_handle);
    /* ESP_LOGI("motor", "Target frequency reached and stabilized."); */
  }

  /* ESP_LOGI("motor", "Freq: %.2f Hz | Target: %.2f Hz | Vel: %.2f Hz/s", */
  /*          *current_freq, *target_freq, *velocity); */

  motor_delete_pwm(motor);
  if (*current_freq > 0.0f) {
    motor_create_pwm(motor);
  } else {
    ESP_LOGI("motor", "PWM stopped at 0 Hz");
  }
}
