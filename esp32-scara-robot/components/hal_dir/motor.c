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

void motor_stop_pwm(motor_t *motor) {
  if (motor == NULL || motor->mcpwm_vars->timer == NULL) {
    return;
  }

  // Check if timer is already stopped/disabled to avoid errors
  esp_err_t err;

  // Try to stop the timer first - only if it's not already stopped
  err =
      mcpwm_timer_start_stop(motor->mcpwm_vars->timer, MCPWM_TIMER_STOP_EMPTY);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "Failed to stop timer: %s", esp_err_to_name(err));
  }

  // Try to disable the timer - only if it's not already disabled
  err = mcpwm_timer_disable(motor->mcpwm_vars->timer);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "Failed to disable timer: %s", esp_err_to_name(err));
  }

  // Force GPIO to LOW to eliminate noise
  gpio_set_level(motor->gpio_stp, 0);

  /* ESP_LOGI(TAG, "PWM cleanly stopped, GPIO set to LOW"); */
}

void motor_delete_pwm(motor_t *motor) {
  if (motor == NULL) {
    ESP_LOGW(TAG, "Pointer is null in motor_delete_pwm");
    return;
  }

  bool cleanup_gpio = false;

  // Stop timer first if it exists
  if (motor->mcpwm_vars->timer) {
    // Try to stop first, but silently ignore "already stopped" errors
    mcpwm_timer_start_stop(motor->mcpwm_vars->timer, MCPWM_TIMER_STOP_EMPTY);

    // Try to disable, but silently ignore "already disabled" errors
    mcpwm_timer_disable(motor->mcpwm_vars->timer);

    // Now delete the timer
    esp_err_t err = mcpwm_del_timer(motor->mcpwm_vars->timer);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Error deleting timer: %s", esp_err_to_name(err));
    }
    motor->mcpwm_vars->timer = NULL;
  }

  if (motor->mcpwm_vars->generator) {
    esp_err_t err = mcpwm_del_generator(motor->mcpwm_vars->generator);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Error deleting generator: %s", esp_err_to_name(err));
    }
    motor->mcpwm_vars->generator = NULL;
  }

  if (motor->mcpwm_vars->comparator) {
    esp_err_t err = mcpwm_del_comparator(motor->mcpwm_vars->comparator);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Error deleting comparator: %s", esp_err_to_name(err));
    }
    motor->mcpwm_vars->comparator = NULL;
  }

  if (motor->mcpwm_vars->operator) {
    esp_err_t err = mcpwm_del_operator(motor->mcpwm_vars->operator);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Error deleting operator: %s", esp_err_to_name(err));
    }
    motor->mcpwm_vars->operator= NULL;
    cleanup_gpio =
        true; // Only cleanup GPIO if operator was successfully deleted
  }

  if (cleanup_gpio) {
    gpio_reset_pin(motor->gpio_stp);
    gpio_set_direction(motor->gpio_stp, GPIO_MODE_OUTPUT);
    gpio_set_level(motor->gpio_stp, 0);
  }
}

void motor_create_pwm(motor_t *motor) {
  if (motor == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    return;
  }

  motor_delete_pwm(motor);

  // For initialization, don't create PWM if frequency is 0 or too low
  if (motor->pwm_vars->current_freq_hz <= 0.0f) {
    /* ESP_LOGI(TAG, "PWM creation skipped for motor %d (freq = %.2f Hz)", */
    /*          motor->id, motor->pwm_vars->current_freq_hz); */
    return;
  }

  if (motor->pwm_vars->current_freq_hz < motor->pwm_vars->min_freq) {
    /* ESP_LOGW(TAG, "Skipping PWM creation: freq = %.2f is too low", */
    /*          motor->pwm_vars->current_freq_hz); */
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

esp_err_t motor_set_target_frequency(motor_t *motor, float target_freq_hz) {
  if (motor == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Set the target frequency - the timer callback will handle acceleration
  motor->pwm_vars->target_freq_hz = target_freq_hz;

  /* ESP_LOGI(TAG, "Target frequency set to %.2f Hz (current: %.2f Hz)", */
  /*          target_freq_hz, motor->pwm_vars->current_freq_hz); */

  // If timer is not running and we have a non-zero target, start it
  if (target_freq_hz != motor->pwm_vars->current_freq_hz) {
    // Start the acceleration timer if not already running
    esp_err_t err = esp_timer_start_periodic(
        motor->mcpwm_vars->esp_timer_handle,
        UPDATE_INTERVAL_MS * 1000); // Convert to microseconds
    if (err == ESP_ERR_INVALID_STATE) {
      // Timer already running, which is fine
      err = ESP_OK;
    }
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start acceleration timer: %s",
               esp_err_to_name(err));
      return err;
    }
  }

  return ESP_OK;
}

// Improved frequency update function with better boundary handling
esp_err_t motor_update_pwm_frequency_immediate(motor_t *motor,
                                               float new_freq_hz) {
  if (motor == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Handle zero frequency - completely delete PWM for clean stop
  if (new_freq_hz <= 0.0f) {
    motor_stop_pwm(motor);
    motor_delete_pwm(motor);
    motor->pwm_vars->current_freq_hz = 0.0f;
    /* ESP_LOGI(TAG, "PWM stopped and deleted for 0 Hz"); */
    return ESP_OK;
  }

  // Check if PWM needs to be created (transitioning from 0 to non-zero)
  if (motor->mcpwm_vars->timer == NULL) {
    esp_err_t err = motor_create_pwm_with_frequency(motor, new_freq_hz);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to recreate PWM");
      return err;
    }
    motor->pwm_vars->current_freq_hz = new_freq_hz;
    return ESP_OK;
  }

  // PWM exists, do efficient in-place update
  float freq = new_freq_hz;

  // Clamp frequency to safe range
  if (freq > motor->pwm_vars->max_freq)
    freq = motor->pwm_vars->max_freq;
  if (freq < motor->pwm_vars->min_freq)
    freq = motor->pwm_vars->min_freq;

  // Calculate new period with improved boundary checking
  uint32_t period_ticks =
      (uint32_t)(motor->mcpwm_vars->pwm_resolution_hz / freq);

  // Ensure period_ticks is within safe bounds with some margin
  uint32_t safe_min_ticks = motor->mcpwm_vars->mcpwm_min_period_ticks + 10;
  uint32_t safe_max_ticks = motor->mcpwm_vars->mcpwm_max_period_ticks - 10;

  if (period_ticks < safe_min_ticks) {
    period_ticks = safe_min_ticks;
    // Recalculate actual frequency based on clamped period
    freq = (float)motor->mcpwm_vars->pwm_resolution_hz / period_ticks;
  }
  if (period_ticks > safe_max_ticks) {
    period_ticks = safe_max_ticks;
    // Recalculate actual frequency based on clamped period
    freq = (float)motor->mcpwm_vars->pwm_resolution_hz / period_ticks;
  }

  uint32_t duty_ticks = period_ticks / 2; // 50% duty cycle

  // Disable timer briefly for atomic update
  esp_err_t err =
      mcpwm_timer_start_stop(motor->mcpwm_vars->timer, MCPWM_TIMER_STOP_EMPTY);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "Warning: Failed to stop timer for update: %s",
             esp_err_to_name(err));
  }

  // Update timer period
  err = mcpwm_timer_set_period(motor->mcpwm_vars->timer, period_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update timer period: %s", esp_err_to_name(err));
    // Try to restart timer even if period update failed
    mcpwm_timer_start_stop(motor->mcpwm_vars->timer, MCPWM_TIMER_START_NO_STOP);
    return err;
  }

  // Update comparator value for 50% duty
  err = mcpwm_comparator_set_compare_value(motor->mcpwm_vars->comparator,
                                           duty_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update compare value: %s", esp_err_to_name(err));
    // Try to restart timer even if comparator update failed
    mcpwm_timer_start_stop(motor->mcpwm_vars->timer, MCPWM_TIMER_START_NO_STOP);
    return err;
  }

  // Restart timer
  err = mcpwm_timer_start_stop(motor->mcpwm_vars->timer,
                               MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to restart timer: %s", esp_err_to_name(err));
    return err;
  }

  motor->pwm_vars->current_freq_hz = freq;
  return ESP_OK;
}

esp_err_t motor_restart_pwm(motor_t *motor) {
  if (motor == NULL || motor->mcpwm_vars->timer == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Re-enable and start timer
  esp_err_t err = mcpwm_timer_enable(motor->mcpwm_vars->timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to re-enable timer: %s", esp_err_to_name(err));
    return err;
  }

  err = mcpwm_timer_start_stop(motor->mcpwm_vars->timer,
                               MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to restart timer: %s", esp_err_to_name(err));
    return err;
  }

  return ESP_OK;
}

// Updated timer callback with fixed step counting
void motor_update_timer_cb(void *arg) {
  motor_t *motor = (motor_t *)arg;

  const float dt = UPDATE_INTERVAL_MS / 1000.0f;
  float current_freq = motor->pwm_vars->current_freq_hz;
  float target_freq = motor->pwm_vars->target_freq_hz;
  float *velocity = &motor->pwm_vars->velocity_hz_per_s;
  float max_accel = motor->pwm_vars->max_accel;

  // STEP COUNTING - Check if we've reached target steps BEFORE updating
  // frequency
  if (motor->pwm_vars->step_counting_enabled && current_freq > 0) {
    // Calculate steps taken this interval based on CURRENT frequency
    float steps_this_interval_f = current_freq * dt;
    uint32_t steps_this_interval = (uint32_t)steps_this_interval_f;

    motor_update_current_position(motor);

    // Add fractional part tracking for better accuracy
    static float fractional_steps = 0.0f;
    fractional_steps += steps_this_interval_f - steps_this_interval;
    if (fractional_steps >= 1.0f) {
      steps_this_interval += (uint32_t)fractional_steps;
      fractional_steps -= (uint32_t)fractional_steps;
    }

    motor->pwm_vars->step_count += steps_this_interval;

    // Check if we've reached or will exceed target steps
    if (motor->pwm_vars->target_steps > 0 &&
        motor->pwm_vars->step_count >= motor->pwm_vars->target_steps) {

      /* ESP_LOGI(TAG, "Motor %d reached target steps: %d (actual: %d)",
       * motor->id, */
      /*          motor->pwm_vars->target_steps, motor->pwm_vars->step_count);
       */

      // IMMEDIATELY stop the motor
      motor->pwm_vars->target_freq_hz = 0;
      motor->pwm_vars->current_freq_hz = 0;
      *velocity = 0;

      // Stop PWM immediately
      motor_stop_pwm(motor);
      motor_delete_pwm(motor);

      // Stop the timer
      esp_timer_stop(motor->mcpwm_vars->esp_timer_handle);

      // Disable step counting to prevent further triggers
      motor->pwm_vars->step_counting_enabled = false;

      ESP_LOGI(TAG, "Motor %d stopped after %d steps", motor->id,
               motor->pwm_vars->step_count);
      return;
    }

    // If we're getting close to target, start decelerating
    if (motor->pwm_vars->target_steps > 0) {
      uint32_t remaining_steps =
          motor->pwm_vars->target_steps - motor->pwm_vars->step_count;

      // Calculate deceleration distance (steps needed to stop from current
      // frequency)
      float decel_time = current_freq / max_accel; // Time to decelerate to 0
      uint32_t decel_steps =
          (uint32_t)(current_freq * decel_time / 2); // Area under triangle

      // If we're within deceleration distance, reduce target frequency
      if (remaining_steps <= decel_steps && remaining_steps > 0) {
        // Calculate what frequency we should have to stop in remaining steps
        float ideal_freq = sqrt(2.0f * max_accel * remaining_steps);
        if (ideal_freq < target_freq) {
          target_freq = ideal_freq;
          motor->pwm_vars->target_freq_hz = target_freq;
        }
      }
    }
  }

  // Calculate frequency error
  float freq_error = target_freq - current_freq;

  // Check if we're close enough to target (within tolerance)
  if (fabs(freq_error) < 0.5f && fabs(*velocity) < 1.0f) {
    motor->pwm_vars->current_freq_hz = target_freq;
    *velocity = 0;

    // Handle target frequency
    if (target_freq > 0.0f) {
      esp_err_t err = motor_update_pwm_frequency_immediate(motor, target_freq);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update PWM frequency to target");
      }
    } else {
      // Target is 0 - delete PWM completely (this will also stop it)
      motor_delete_pwm(motor);
      ESP_LOGI(TAG, "PWM stopped and deleted for 0 Hz target");

      // Stop the timer - target reached
      esp_timer_stop(motor->mcpwm_vars->esp_timer_handle);
      return;
    }

    // If not doing step counting, stop the timer when target is reached
    if (!motor->pwm_vars->step_counting_enabled ||
        motor->pwm_vars->target_steps <= 0) {
      esp_timer_stop(motor->mcpwm_vars->esp_timer_handle);
      return;
    }
  }

  // Calculate desired acceleration based on error
  float desired_accel;
  if (freq_error > 0) {
    desired_accel = max_accel; // Accelerate toward target
  } else {
    desired_accel = -max_accel; // Decelerate toward target
  }

  // Apply acceleration limits with better control
  float accel_step = desired_accel * dt;
  *velocity += accel_step;

  // Limit maximum velocity to prevent overshoot
  float max_velocity = max_accel * 2.0f;
  if (*velocity > max_velocity)
    *velocity = max_velocity;
  if (*velocity < -max_velocity)
    *velocity = -max_velocity;

  // Update current frequency
  float new_freq = current_freq + (*velocity * dt);

  // Prevent overshooting the target
  if ((freq_error > 0 && new_freq > target_freq) ||
      (freq_error < 0 && new_freq < target_freq)) {
    new_freq = target_freq;
    *velocity = 0;
  }

  // Ensure frequency doesn't go negative
  if (new_freq < 0) {
    new_freq = 0;
    *velocity = 0;
  }

  // Update the motor's current frequency
  motor->pwm_vars->current_freq_hz = new_freq;

  // Handle frequency transitions more smoothly
  if (new_freq > 0.0f) {
    // Check if PWM needs to be created (transitioning from 0)
    if (motor->mcpwm_vars->timer == NULL) {
      // Create PWM with minimum safe frequency first, then update
      float safe_start_freq = motor->pwm_vars->min_freq;
      esp_err_t err = motor_create_pwm_with_frequency(motor, safe_start_freq);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PWM during transition");
        esp_timer_stop(motor->mcpwm_vars->esp_timer_handle);
        return;
      }
      // Small delay to ensure PWM is stable before updating frequency
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Now update to the actual target frequency
    esp_err_t err = motor_update_pwm_frequency_immediate(motor, new_freq);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update PWM frequency");
      // On error, stop the acceleration timer to prevent further issues
      esp_timer_stop(motor->mcpwm_vars->esp_timer_handle);
      return;
    }
  } else if (new_freq == 0.0f && motor->mcpwm_vars->timer != NULL) {
    // Frequency reached 0 - first stop PWM cleanly, then delete
    motor_stop_pwm(motor);
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for clean stop
    motor_delete_pwm(motor);
    /* ESP_LOGI(TAG, "PWM stopped and deleted at 0 Hz"); */
  }
}

esp_err_t motor_init_timer(motor_t *motor) {
  if (motor == NULL) {
    ESP_LOGE(TAG, "Motor pointer is null in motor_init_timer");
    return ESP_ERR_INVALID_ARG;
  }

  // Timer configuration
  esp_timer_create_args_t timer_args = {
      .callback = motor_update_timer_cb,
      .arg = motor,
      .name = motor->label, // Use motor label as timer name
      .dispatch_method = ESP_TIMER_TASK,
      .skip_unhandled_events = true};

  esp_err_t err =
      esp_timer_create(&timer_args, &motor->mcpwm_vars->esp_timer_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create timer for %s: %s", motor->label,
             esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "Timer created successfully for %s", motor->label);
  return ESP_OK;
}
// New function to create PWM with a specific frequency
esp_err_t motor_create_pwm_with_frequency(motor_t *motor, float freq_hz) {
  if (motor == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    return ESP_ERR_INVALID_ARG;
  }

  motor_delete_pwm(motor);

  // Skip creation if frequency is 0 or too low
  if (freq_hz <= 0.0f) {
    ESP_LOGI(TAG, "PWM creation skipped for motor %d (freq = %.2f Hz)",
             motor->id, freq_hz);
    return ESP_OK;
  }

  if (freq_hz < motor->pwm_vars->min_freq) {
    ESP_LOGW(TAG, "Skipping PWM creation: freq = %.2f is too low", freq_hz);
    return ESP_ERR_INVALID_ARG;
  }

  // Clamp frequency to a safe range
  float freq = freq_hz;
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

  esp_err_t err = mcpwm_new_timer(&timer_config, &motor->mcpwm_vars->timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(err));
    return err;
  }

  // Create operator and connect timer
  mcpwm_operator_config_t operator_config = {
      .group_id = motor->mcpwm_vars->group_unit,
  };

  err = mcpwm_new_operator(&operator_config, &motor->mcpwm_vars->operator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM operator: %s", esp_err_to_name(err));
    return err;
  }

  err = mcpwm_operator_connect_timer(motor->mcpwm_vars->operator,
                                     motor->mcpwm_vars->timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to connect operator to timer: %s",
             esp_err_to_name(err));
    return err;
  }

  // Create comparator
  mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez =
                                                     true};
  err = mcpwm_new_comparator(motor->mcpwm_vars->operator, & comparator_config,
                             &motor->mcpwm_vars->comparator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create comparator: %s", esp_err_to_name(err));
    return err;
  }

  err = mcpwm_comparator_set_compare_value(motor->mcpwm_vars->comparator,
                                           duty_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set compare value: %s", esp_err_to_name(err));
    return err;
  }

  // Create generator
  mcpwm_generator_config_t generator_config = {.gen_gpio_num = motor->gpio_stp};
  err = mcpwm_new_generator(motor->mcpwm_vars->operator, & generator_config,
                            &motor->mcpwm_vars->generator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create generator: %s", esp_err_to_name(err));
    return err;
  }

  err = mcpwm_generator_set_action_on_timer_event(
      motor->mcpwm_vars->generator,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY,
                                   MCPWM_GEN_ACTION_HIGH));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator high action: %s",
             esp_err_to_name(err));
    return err;
  }

  err = mcpwm_generator_set_action_on_compare_event(
      motor->mcpwm_vars->generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     motor->mcpwm_vars->comparator,
                                     MCPWM_GEN_ACTION_LOW));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator low action: %s",
             esp_err_to_name(err));
    return err;
  }

  // Enable and start timer
  err = mcpwm_timer_enable(motor->mcpwm_vars->timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(err));
    return err;
  }

  err = mcpwm_timer_start_stop(motor->mcpwm_vars->timer,
                               MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    return err;
  }

  /* ESP_LOGI(TAG, "PWM created for motor %d at freq %.2f Hz (period %u ticks)",
   */
  /*          motor->id, freq, period_ticks); */
  return ESP_OK;
}

void motor_reset_step_count(motor_t *motor) {
  if (motor == NULL)
    return;
  motor->pwm_vars->step_count = 0;
  ESP_LOGI(TAG, "Step count reset for motor %d", motor->id);
}

void motor_enable_step_counting(motor_t *motor, bool enable) {
  if (motor == NULL)
    return;
  motor->pwm_vars->step_counting_enabled = enable;
  ESP_LOGI(TAG, "Step counting %s for motor %d",
           enable ? "enabled" : "disabled", motor->id);
}

esp_err_t motor_set_target_steps(motor_t *motor, int target_steps) {
  if (motor == NULL)
    return ESP_ERR_INVALID_ARG;
  motor->pwm_vars->target_steps = target_steps;
  ESP_LOGI(TAG, "Target steps set to %d for motor %d", target_steps, motor->id);
  return ESP_OK;
}

int motor_get_step_count(motor_t *motor) {
  if (motor == NULL)
    return -1;
  return motor->pwm_vars->step_count;
}

esp_err_t motor_move_steps(motor_t *motor, int steps, float frequency) {
  if (motor == NULL)
    return ESP_ERR_INVALID_ARG;

  // Stop any current movement first
  motor_set_target_frequency(motor, 0);
  vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to ensure stop

  // Enable step counting
  motor->pwm_vars->step_counting_enabled = true;

  // Reset counter and set target
  motor->pwm_vars->step_count = 0;
  motor->pwm_vars->target_steps = abs(steps);

  // Set direction based on positive/negative steps
  bool forward = (steps >= 0);
  if (motor->is_inverted)
    forward = !forward;
  gpio_set_level(motor->gpio_dir, forward ? 1 : 0);

  ESP_LOGI(TAG, "Moving motor %d: %d steps at %.2f Hz", motor->id, steps,
           frequency);

  // Start movement
  return motor_set_target_frequency(motor, frequency);
}

esp_err_t motor_move_to_position(motor_t *motor, int target_position,
                                 float frequency) {
  if (motor == NULL)
    return ESP_ERR_INVALID_ARG;

  // Calculate steps needed to reach target position
  int current_position =
      motor->pwm_vars
          ->current_position; // You need to add this to your motor struct
  int steps_to_move = target_position - current_position;

  ESP_LOGI(TAG, "Motor %d: Moving from position %d to %d (%d steps)", motor->id,
           current_position, target_position, steps_to_move);

  // If we're already at the target position, no need to move
  if (steps_to_move == 0) {
    ESP_LOGI(TAG, "Motor %d already at target position %d", motor->id,
             target_position);
    return ESP_OK;
  }

  // Update target position before moving
  motor->pwm_vars->target_position = target_position;

  // Use the existing motor_move_steps function
  esp_err_t result = motor_move_steps(motor, steps_to_move, frequency);

  if (result == ESP_OK) {
    // Update current position after successful move command
    // Note: The actual position will be updated when the move completes
    ESP_LOGI(TAG, "Motor %d move command sent successfully", motor->id);
  }

  return result;
}

// You'll also need a function to update the current position when moves
// complete
void motor_update_current_position(motor_t *motor) {
  if (motor == NULL)
    return;

  // This should be called when a move completes
  // You might want to call this from your timer callback when target steps are
  // reached
  if (motor->pwm_vars->target_position != motor->pwm_vars->current_position) {
    motor->pwm_vars->current_position = motor->pwm_vars->target_position;
    ESP_LOGI(TAG, "Motor %d position updated to %d", motor->id,
             motor->pwm_vars->current_position);
  }
}

// Add this to your motor initialization
esp_err_t motor_set_current_position(motor_t *motor, int position) {
  if (motor == NULL)
    return ESP_ERR_INVALID_ARG;

  motor->pwm_vars->current_position = position;
  motor->pwm_vars->target_position = position;
  ESP_LOGI(TAG, "Motor %d current position set to %d", motor->id, position);
  return ESP_OK;
}
