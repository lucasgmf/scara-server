#include "scara_controller.h"

static const char *TAG = "scara_controller";

//////////////////////////////////
////// network/wifi_manager //////
//////////////////////////////////

wifi_config_t wifi_config_a = {
    .sta =
        {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
};

wifi_rec_data wifi_received_data = {
    .Kp = 1.2f,
    .Ki = 0.001f,
    .Kd = 0.1f,
    .target_position_1 = 2000,
    .target_position_2 = 2000,
};

network_configuration esp_net_conf = {
    .wifi_config = &wifi_config_a,
    .port = PORT,
    .retry_num = 0,
    .rx_buffer_size = RX_BUFFER_SIZE,
    .addr_str_size = ADDR_STR_SIZE,
    .rec_data = &wifi_received_data,
    .s_wifi_event_group = (EventGroupHandle_t)&wifi_received_data,
};

void wifi_initialization_func() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  esp_err_t wifi_result = init_wifi(&esp_net_conf);
  if (wifi_result != ESP_OK) {
    ESP_LOGE(TAG, "Exiting: Wi-Fi connection failed.");
    esp_deep_sleep_start();
    return;
  }
  xTaskCreate(tcp_server_task, "tcp_server", 4096, &esp_net_conf, 5, NULL);
  return;
}

//////////////////////////////////
////// drivers/i2c_bus ///////////
//////////////////////////////////

static SemaphoreHandle_t i2c_mutex;

static i2c_master_bus_handle_t master_bus_handle;
static i2c_master_bus_config_t master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .sda_io_num = GPIO_I2C_SDA,
    .scl_io_num = GPIO_I2C_SCL,
    .glitch_ignore_cnt = I2C_MASTER_GLITCH_IGNORE_CNT,
    .flags.enable_internal_pullup = I2C_ENABLE_INTERNAL_PULLUP,
};

static i2c_master_config_t i2c_master_conf = {
    .bus_cfg = &master_bus_config,
    .bus_handle = &master_bus_handle,
    .i2c_mutex = &i2c_mutex,
    .timeout_ticks = portMAX_DELAY,
};

static i2c_master_dev_handle_t encoder_0_handle;
static i2c_master_dev_handle_t encoder_1_handle;
static i2c_master_dev_handle_t tca_handle;

static i2c_device_config_t encoder_0_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_AS5600,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};

static i2c_device_config_t encoder_1_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_AS5600,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};

static i2c_device_config_t tca_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_TCA,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};

static i2c_slave_bus_params i2c_slave_conf_encoder_0 = {
    .dev_cfg = &encoder_0_cfg,
    .dev_handle = &encoder_0_handle,
};

static i2c_slave_bus_params i2c_slave_conf_encoder_1 = {
    .dev_cfg = &encoder_1_cfg,
    .dev_handle = &encoder_1_handle,
};

static i2c_slave_bus_params i2c_slave_conf_tca = {
    .dev_cfg = &tca_cfg,
    .dev_handle = &tca_handle,
};

static encoder_t encoder_0 = {
    .label = "Encoder 0",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_0,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 0,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,
    .offset = 0,
    .reverse = false,
    .current_reading = 0,
};

// TODO: FIX acceleration in the 0 transition!

/* static encoder_t encoder_1 = { */
/*     .label = "Encoder 1", */
/*     .i2c_master = &i2c_master_conf, */
/*     .i2c_slave = &i2c_slave_conf_encoder_1, */
/*     .i2c_tca = &i2c_slave_conf_tca, */
/*     .tca_channel = 1, */
/*     .reg_angle_msb = ENCODER_MSB_ANGLE_REG, */
/*     .reg_angle_mask = ENCODER_ANGLE_MASK, */
/*     .offset = 0, */
/*     .reverse = false, */
/* }; */

void encoder_initialization_task() {
  i2c_mutex = xSemaphoreCreateMutex();

  ESP_ERROR_CHECK(
      i2c_new_master_bus(i2c_master_conf.bus_cfg, i2c_master_conf.bus_handle));

  ESP_ERROR_CHECK(i2c_master_bus_add_device(*i2c_master_conf.bus_handle,
                                            i2c_slave_conf_tca.dev_cfg,
                                            i2c_slave_conf_tca.dev_handle));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_0.tca_channel, encoder_0.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_0.dev_cfg,
      i2c_slave_conf_encoder_0.dev_handle));

  ESP_ERROR_CHECK(encoder_init(&encoder_0));
  xTaskCreate(encoder_task, "encoder0_task", 4096, &encoder_0, 5, NULL);
}

//////////////////////////////////
////// hal_dir/motor /////////////
//////////////////////////////////

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

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

typedef struct {
  encoder_t *ref_encoder;
  float encoder_target_pos;

  bool enable_pid;
  pid_controller_t *pid;
} motor_control_vars;

typedef struct {
  const char *label;
  int id;
  gpio_num_t gpio_stp;
  gpio_num_t gpio_dir;
  motor_mcpwm_vars *mcpwm_vars;
  motor_pwm_vars_t *pwm_vars;
  motor_control_vars *control_vars;
} motor_t;

#define MOTOR_X_LABEL "Motor x"
#define MOTOR_X_ID 0

#define MCPWM_MAX_PERIOD_TICKS 60000 // WARN: maybe change this
#define MCPWM_MIN_PERIOD_TICKS 5
#define PWM_RESOLUTION_HZ 1000000
#define PWM_MAX_FREQ_HZ 1100
#define PWM_MIN_FREQ_HZ 1
#define PWM_MAX_ACCEL_HZ 3000

motor_mcpwm_vars mcpwm_vars_x = {
    .group_unit = 0,
    .timer = 0,
    .operator= 0,
    .comparator = 0,
    .generator = 0,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_pwm_vars_t pwm_vars_x = {
    .step_count = 0,
    .max_freq = PWM_MAX_FREQ_HZ,
    .min_freq = PWM_MIN_FREQ_HZ,
    .max_accel = PWM_MAX_ACCEL_HZ,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

pid_controller_t pid_x = {
    .Kp = 1,    // 1
    .Ki = 0.005 * 0.5, // 0.01
    .Kd = 0.2,  // 0.2
};

motor_control_vars control_vars_x = {
    .ref_encoder = &encoder_0,
    .encoder_target_pos = 0,
    .enable_pid = true,
    .pid = &pid_x,
};

motor_t motor_x = {
    .label = MOTOR_X_LABEL,
    .id = MOTOR_X_ID,
    .gpio_stp = GPIOXSTP,
    .gpio_dir = GPIOXDIR,
    .mcpwm_vars = &mcpwm_vars_x,
    .pwm_vars = &pwm_vars_x,
    .control_vars = &control_vars_x,
};

void motor_init_dir(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW(TAG, "Pointer is null in motor_init_dir\n");
    return;
  }
  gpio_reset_pin(motor_n->gpio_dir);
  gpio_set_direction(motor_n->gpio_dir, GPIO_MODE_OUTPUT);
  ESP_LOGW(TAG, "Successfully initializated dir from motor %d", motor_n->id);

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

#define UPDATE_INTERVAL_MS 100 // timer period

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
  /* ESP_LOGW("motor_set_frequency", "just set frequency to %f",
   * target_freq_hz); */
}

#define MOTOR_CONTROL_TASK_PERIOD_MS 20

void motor_control_task(void *arg) {
  motor_t *motor = (motor_t *)arg;
  if (motor == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  float error = 0;
  float output = 0;
  float local_target_freq_hz = 0;

  while (1) {
    error = motor->control_vars->encoder_target_pos -
            motor->control_vars->ref_encoder->current_reading;

    const float DEAD_BAND_THRESHOLD = 2.0f;            // WARN: define this
    float dt = MOTOR_CONTROL_TASK_PERIOD_MS / 1000.0f; // Time in seconds

    // Deadband: stop motor and prevent integral windup
    if (fabsf(error) < DEAD_BAND_THRESHOLD) {
      motor->control_vars->pid->integral = 0.0f;
      motor->pwm_vars->target_freq_hz = 0.0f;
      motor_set_frequency(motor, 0.0f);
      vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
      continue;
    }

    // PID calculations
    motor->control_vars->pid->integral += error * dt;

    // Optional: limit the integral term to prevent windup
    const float INTEGRAL_LIMIT = 2000.0f * 4;
    if (motor->control_vars->pid->integral > INTEGRAL_LIMIT)
      motor->control_vars->pid->integral = INTEGRAL_LIMIT;
    else if (motor->control_vars->pid->integral < -INTEGRAL_LIMIT)
      motor->control_vars->pid->integral = -INTEGRAL_LIMIT;

    float derivative = (error - motor->control_vars->pid->prev_error) / dt;
    motor->control_vars->pid->prev_error = error;

    output = motor->control_vars->pid->Kp * error +
             motor->control_vars->pid->Ki * motor->control_vars->pid->integral +
             motor->control_vars->pid->Kd * derivative;

    ESP_LOGI("PID",
             "error: %.2f - output: %.2f | "
             "Kp*error = %.2f, Ki* integral = %.2f, Kd * derivative = %.2f",

             error, output, motor->control_vars->pid->Kp * error,
             motor->control_vars->pid->Ki * motor->control_vars->pid->integral,
             motor->control_vars->pid->Kd * derivative);

    // Clamp output to allowed frequency
    if (output > motor->pwm_vars->max_freq)
      output = motor->pwm_vars->max_freq;
    if (output < -motor->pwm_vars->max_freq)
      output = -motor->pwm_vars->max_freq;

    // WARN: maybe this is target_freq_hz??
    /* loop->output_freq_hz = fabsf(output); */

    /* ESP_LOGI("motor_control_task", "saving target freq to %f", fabs(output));
     */
    local_target_freq_hz = fabsf(output);

    // Determine direction
    bool reverse = output < 0;
    gpio_set_level(motor->gpio_dir,
                   motor->pwm_vars->dir_is_reversed ? !reverse : reverse);

    // Apply new frequency to motor
    motor_set_frequency(motor, local_target_freq_hz);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
  }
}

void motor_initialization_task() {
  motor_init_dir(&motor_x);
  motor_create_pwm(&motor_x);

  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_x, 5, NULL);
  return;
}

void init_scara() {
  /* wifi_initialization_func(); */
  encoder_initialization_task();
  motor_initialization_task();

  ESP_LOGI(TAG, "");
  return;
}

void loop_scara() {
  while (1) {
    motor_x.control_vars->encoder_target_pos = 0 + 250;
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    motor_x.control_vars->encoder_target_pos = 4096 - 250;
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    /* motor_x.control_vars->pid->Kp = wifi_received_data.Kp; */
    /* motor_x.control_vars->pid->Ki = wifi_received_data.Ki; */
    /* motor_x.control_vars->pid->Kd = wifi_received_data.Kd; */
    /**/
    /* motor_x.control_vars->encoder_target_pos = */
    /*     wifi_received_data.target_position_1; */
    /* vTaskDelay(5000 / portTICK_PERIOD_MS); */
    /**/
    /* motor_x.control_vars->encoder_target_pos = */
    /*     wifi_received_data.target_position_2; */
    /* vTaskDelay(5000 / portTICK_PERIOD_MS); */
  }

  return;
}
