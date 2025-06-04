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
    ESP_LOGW(TAG, "Pointer is null in motor_delete_pwm");
    return;
  }

  bool cleanup_gpio = false;

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
    cleanup_gpio =
        true; // Only cleanup GPIO if operator was successfully deleted
  }

  // Only reset GPIO after successful cleanup
  if (cleanup_gpio) {
    gpio_reset_pin(motor->gpio_stp);
    gpio_set_direction(motor->gpio_stp, GPIO_MODE_OUTPUT);
    gpio_set_level(motor->gpio_stp, 0);
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

  ESP_LOGI(TAG, "PWM created for motor %d at freq %.2f Hz (period %u ticks)",
           motor->id, freq, period_ticks);
}

esp_err_t motor_update_pwm_frequency(motor_t *motor, float new_freq_hz) {
  if (motor == NULL || motor->mcpwm_vars->timer == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Clamp frequency to safe range
  float freq = new_freq_hz;
  if (freq > motor->pwm_vars->max_freq)
    freq = motor->pwm_vars->max_freq;
  if (freq < motor->pwm_vars->min_freq)
    freq = motor->pwm_vars->min_freq;

  // Calculate new period
  uint32_t period_ticks =
      (uint32_t)(motor->mcpwm_vars->pwm_resolution_hz / freq);
  if (period_ticks < motor->mcpwm_vars->mcpwm_min_period_ticks)
    period_ticks = motor->mcpwm_vars->mcpwm_min_period_ticks;
  if (period_ticks > motor->mcpwm_vars->mcpwm_max_period_ticks)
    period_ticks = motor->mcpwm_vars->mcpwm_max_period_ticks;

  uint32_t duty_ticks = period_ticks / 2; // 50% duty cycle

  // Update timer period (this is the key optimization!)
  esp_err_t err =
      mcpwm_timer_set_period(motor->mcpwm_vars->timer, period_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update timer period: %s", esp_err_to_name(err));
    return err;
  }

  // Update comparator value for 50% duty
  err = mcpwm_comparator_set_compare_value(motor->mcpwm_vars->comparator,
                                           duty_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update compare value: %s", esp_err_to_name(err));
    return err;
  }

  return ESP_OK;
}

void motor_update_timer_cb(void *arg) {
  motor_t *motor = (motor_t *)arg;
  const float dt = UPDATE_INTERVAL_MS / 1000.0f;
  float *current_freq = &motor->pwm_vars->current_freq_hz;
  float *target_freq = &motor->pwm_vars->target_freq_hz;
  float *velocity = &motor->pwm_vars->velocity_hz_per_s;
  float max_accel = motor->pwm_vars->max_accel;

  // Your existing frequency calculation logic...
  float freq_error = *target_freq - *current_freq;
  float desired_velocity = freq_error / dt;
  float accel_step = max_accel * dt;

  if (desired_velocity > *velocity + accel_step) {
    *velocity += accel_step;
  } else if (desired_velocity < *velocity - accel_step) {
    *velocity -= accel_step;
  } else {
    *velocity = desired_velocity;
  }

  *current_freq += (*velocity * dt);

  if ((freq_error > 0 && *current_freq > *target_freq) ||
      (freq_error < 0 && *current_freq < *target_freq)) {
    *current_freq = *target_freq;
    *velocity = 0;
  }

  // **KEY CHANGE**: Use efficient update instead of recreate
  if (*current_freq > 0.0f) {
    esp_err_t err = motor_update_pwm_frequency(motor, *current_freq);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update PWM frequency");
    }
  } else {
    // Only delete PWM when stopping completely
    motor_delete_pwm(motor);
    ESP_LOGI(TAG, "PWM stopped at 0 Hz");
  }

  // Check if target reached
  if (fabs(*target_freq - *current_freq) < 0.5f && fabs(*velocity) < 1.0f) {
    *current_freq = *target_freq;
    *velocity = 0;
    esp_timer_stop(motor->mcpwm_vars->esp_timer_handle);
  }

  ESP_LOGI(TAG, "Freq: %.2f Hz | Target: %.2f Hz | Vel: %.2f Hz/s",
           *current_freq, *target_freq, *velocity);
}
