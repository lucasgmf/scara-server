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

///////////////////////////
////// switch /////////////
///////////////////////////

#define GPIO_SWITCH_0 GPIO_NUM_15
#define GPIO_SWITCH_1 GPIO_NUM_32

gpio_config_t switch_0_io_conf = {
    .pin_bit_mask = (1ULL << GPIO_SWITCH_0), // Set the GPIO pin
    .mode = GPIO_MODE_INPUT,                 // Set as input mode
    .pull_up_en = GPIO_PULLUP_ENABLE,        // Enable pull-up resistor
    .pull_down_en = GPIO_PULLDOWN_DISABLE,   // Disable pull-down
    .intr_type = GPIO_INTR_DISABLE,
};

gpio_config_t switch_1_io_conf = {
    .pin_bit_mask = (1ULL << GPIO_SWITCH_1),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

switch_t switch_0 = {
    .config = &switch_0_io_conf,
    .gpio_pin = GPIO_SWITCH_0,
    .is_pressed = false,
};

switch_t switch_1 = {
    .config = &switch_1_io_conf,
    .gpio_pin = GPIO_SWITCH_1,
    .is_pressed = false,
};

void switch_initialization_task() {
  switch_init(&switch_0);
  switch_init(&switch_1);

  // TODO: one task for all switch
  xTaskCreate(switch_task, "switch_update_task", 4096, &switch_0, 5, NULL);
  xTaskCreate(switch_task, "switch_update_task", 4096, &switch_1, 5, NULL);
}
//////////////////////////////////
////// drivers/i2c_bus ///////////
//////////////////////////////////

static SemaphoreHandle_t i2c_mutex = NULL;

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
    /* .i2c_mutex = &i2c_mutex, */
    .timeout_ticks = portMAX_DELAY,
};

static i2c_master_dev_handle_t encoder_0_handle;
static i2c_master_dev_handle_t encoder_1_handle;
static i2c_master_dev_handle_t encoder_2_handle;
static i2c_master_dev_handle_t encoder_3_handle;
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
static i2c_device_config_t encoder_2_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_AS5600,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};
static i2c_device_config_t encoder_3_cfg = {
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

static i2c_slave_bus_params i2c_slave_conf_encoder_2 = {
    .dev_cfg = &encoder_2_cfg,
    .dev_handle = &encoder_2_handle,
};

static i2c_slave_bus_params i2c_slave_conf_encoder_3 = {
    .dev_cfg = &encoder_3_cfg,
    .dev_handle = &encoder_3_handle,
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

    .initial_offset = 3906,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 0,
    .is_calibrated = false,
    .switch_n = &switch_0,
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

static encoder_t encoder_1 = {
    .label = "Encoder 1",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_1,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 1,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,

    .initial_offset = -5876,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 706,
    .is_calibrated = false,
    .switch_n = &switch_1,
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

static encoder_t encoder_2 = {
    .label = "Encoder 2",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_2,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 2,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,

    .initial_offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 2,
    .test_offset = 706,
    .is_calibrated = false,
    .switch_n = &switch_1,
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

static encoder_t encoder_3 = {
    .label = "Encoder 3",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_3,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 3,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,

    .initial_offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 1,
    .test_offset = 706,
    .is_calibrated = false,
    /* .switch_n = &switch_1, */
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

#include "stdlib.h"

void encoder_initialization_task() {

  i2c_mutex = xSemaphoreCreateMutex();
  if (i2c_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create I2C mutex!");
    return;
  }

  i2c_master_conf.i2c_mutex = i2c_mutex;

  ESP_ERROR_CHECK(
      i2c_new_master_bus(i2c_master_conf.bus_cfg, i2c_master_conf.bus_handle));
  ESP_LOGI(TAG, "I2C bus handle: %p", *i2c_master_conf.bus_handle);

  ESP_ERROR_CHECK(i2c_master_bus_add_device(*i2c_master_conf.bus_handle,
                                            i2c_slave_conf_tca.dev_cfg,
                                            i2c_slave_conf_tca.dev_handle));

  ESP_LOGI(TAG, "TCA handle pointer: %p", (void *)encoder_0.i2c_tca);
  ESP_LOGI(TAG, "TCA device handle: %p", (void *)encoder_0.i2c_tca->dev_handle);

  if (encoder_0.i2c_tca != NULL && encoder_0.i2c_tca->dev_handle != NULL) {
    ESP_ERROR_CHECK(tca_select_channel(encoder_0.tca_channel,
                                       encoder_0.i2c_tca->dev_handle));
  } else {
    ESP_LOGE(TAG, "TCA device handle not initialized yet.");
  }

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_0.tca_channel, encoder_0.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_0.dev_cfg,
      i2c_slave_conf_encoder_0.dev_handle));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_1.tca_channel, encoder_1.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_1.dev_cfg,
      i2c_slave_conf_encoder_1.dev_handle));
  ESP_ERROR_CHECK(encoder_init(&encoder_1));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_2.tca_channel, encoder_2.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_2.dev_cfg,
      i2c_slave_conf_encoder_2.dev_handle));
  ESP_ERROR_CHECK(encoder_init(&encoder_2));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_3.tca_channel, encoder_3.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_3.dev_cfg,
      i2c_slave_conf_encoder_3.dev_handle));
  ESP_ERROR_CHECK(encoder_init(&encoder_3));

  xTaskCreate(encoder_task, "encoder0_task", 4096, &encoder_0, 5, NULL);
  xTaskCreate(encoder_task, "encoder1_task", 4096, &encoder_1, 5, NULL);
  xTaskCreate(encoder_task, "encoder2_task", 4096, &encoder_2, 5, NULL);
  xTaskCreate(encoder_task, "encoder3_task", 4096, &encoder_3, 5, NULL);

  /* xTaskCreate(encoder_try_calibration_task, "encoder0_cal_task", 4096, */
  /*             &encoder_0, 5, NULL); */

  ESP_LOGE(TAG, "exiting encoder_initialization_task");
}

//////////////////////////////////
////// network/wifi_manager //////
//////////////////////////////////

/* Motor	mcpwm_unit	mcpwm_timer	mcpwm_opr */
/* 1	0	0	0 */
/* 2	0	1	1 */
/* 3	0	2	2 */
/* 4	1	0	0 */
/* 5	1	1	1 */
/* 6	1	2	2 */

motor_mcpwm_vars mcpwm_vars_x = {
    .group_unit = 0,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_y = {
    .group_unit = 0,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_z = {
    .group_unit = 1,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_a = {
    .group_unit = 1,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_b = {
    .group_unit = 1,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};
motor_pwm_vars_t pwm_vars_x = {
    .step_count = 0,
    .max_freq = 1200,
    .min_freq = 0,
    .max_accel = 3000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

motor_pwm_vars_t pwm_vars_y = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 3000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

motor_pwm_vars_t pwm_vars_z = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 3000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

motor_pwm_vars_t pwm_vars_a = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 1000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

motor_pwm_vars_t pwm_vars_b = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 5000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

pid_controller_t pid_motor_x = {
    .Kp = 0.2,    // 1
    .Ki = 0.0025, // 0.01
    .Kd = 0.0,    // 0.2
};

pid_controller_t pid_motor_y = {
    .Kp = 0.2 * 4, // 1
    .Ki = 0.0025,  // 0.01
    .Kd = 0.0,     // 0.2
};

pid_controller_t pid_motor_z = {
    .Kp = 0.1 * 5, // 1
    .Ki = 0.0,     // 0.01
    .Kd = 0.0,     // 0.2
};

pid_controller_t pid_motor_a = {
    .Kp = 0.1, // 1
    .Ki = 0.0, // 0.01
    .Kd = 0.0, // 0.2
};

pid_controller_t pid_motor_b = {
    .Kp = 0.1, // 1
    .Ki = 0.0, // 0.01
    .Kd = 0.0, // 0.2
};

motor_control_vars control_vars_x = {
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_y,
    .ref_switch = &switch_0,
};

motor_control_vars control_vars_y = {
    .ref_encoder = &encoder_0,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_y,
    .ref_switch = &switch_1,
};

motor_control_vars control_vars_z = {
    .ref_encoder = &encoder_1,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_z,
    .ref_switch = &switch_1,
};

motor_control_vars control_vars_a = {
    .ref_encoder = &encoder_2,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_a,
    /* .ref_switch = &switch_1, */
};
motor_control_vars control_vars_b = {
    .ref_encoder = &encoder_3,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_b,
    /* .ref_switch = &switch_1, */
};

motor_t motor_x = {
    .label = MOTOR_X_LABEL,
    .id = MOTOR_X_ID,
    .gpio_stp = GPIO_X_STP,
    .gpio_dir = GPIO_X_DIR,
    .mcpwm_vars = &mcpwm_vars_x,
    .pwm_vars = &pwm_vars_x,
    .control_vars = &control_vars_x,
    .is_inverted = true,
};

motor_t motor_y = {
    .label = MOTOR_Y_LABEL,
    .id = MOTOR_Y_ID,
    .gpio_stp = GPIO_Y_STP,
    .gpio_dir = GPIO_Y_DIR,
    .mcpwm_vars = &mcpwm_vars_y,
    .pwm_vars = &pwm_vars_y,
    .control_vars = &control_vars_y,
    .is_inverted = false,
};

motor_t motor_z = {
    .label = MOTOR_Z_LABEL,
    .id = MOTOR_Z_ID,
    .gpio_stp = GPIO_Z_STP,
    .gpio_dir = GPIO_Z_DIR,
    .mcpwm_vars = &mcpwm_vars_z,
    .pwm_vars = &pwm_vars_z,
    .control_vars = &control_vars_z,
    .is_inverted = true,
};

motor_t motor_a = {
    .label = MOTOR_A_LABEL,
    .id = MOTOR_A_ID,
    .gpio_stp = GPIO_A_STP,
    .gpio_dir = GPIO_A_DIR,
    .mcpwm_vars = &mcpwm_vars_a,
    .pwm_vars = &pwm_vars_a,
    .control_vars = &control_vars_a,
    .is_inverted = false,
};
motor_t motor_b = {
    .label = MOTOR_B_LABEL,
    .id = MOTOR_B_ID,
    .gpio_stp = GPIO_B_STP,
    .gpio_dir = GPIO_B_DIR,
    .mcpwm_vars = &mcpwm_vars_b,
    .pwm_vars = &pwm_vars_b,
    .control_vars = &control_vars_b,
    .is_inverted = false,
};

void motor_initialization_task() {
  esp_err_t err;

  motor_init_dir(&motor_x);
  motor_create_pwm(&motor_x);
  err = motor_init_timer(&motor_x);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor X");
  }
  motor_x.pwm_vars->target_steps = 0;
  motor_x.pwm_vars->step_counting_enabled = true;

  motor_init_dir(&motor_y);
  motor_create_pwm(&motor_y);
  err = motor_init_timer(&motor_y);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor Y");
  }

  motor_init_dir(&motor_z);
  motor_create_pwm(&motor_z);
  err = motor_init_timer(&motor_z);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor Z");
  }

  motor_init_dir(&motor_a);
  motor_create_pwm(&motor_a);
  err = motor_init_timer(&motor_a);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor A");
  }
  motor_init_dir(&motor_b);
  motor_create_pwm(&motor_b);
  err = motor_init_timer(&motor_b);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor B");
  }

  ESP_LOGI(TAG, "All motors initialized successfully");
  /* xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_x, 5, NULL); */

  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_y, 5, NULL);
  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_z, 5, NULL);

  /* xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_z, 5, NULL);
   */
  return;
}

void calibration_initialization_task() {
  gpio_set_level(motor_x.gpio_dir, true);
  gpio_set_level(motor_y.gpio_dir, false);
  gpio_set_level(motor_z.gpio_dir, true);
  gpio_set_level(motor_a.gpio_dir, false);
  gpio_set_level(motor_b.gpio_dir, false);

  motor_set_target_frequency(&motor_x, motor_x.pwm_vars->max_freq);
  while (!motor_x.control_vars->ref_switch->is_pressed) {
    ESP_LOGI("calibration_initialization_task", "calibrating motor_x");
    vTaskDelay(20);
  }
  motor_set_target_frequency(&motor_x, 0);
  motor_move_steps(&motor_x, 6400 * 10 / 6 * 5, motor_x.pwm_vars->max_freq);

  motor_set_target_frequency(&motor_y, motor_y.pwm_vars->max_freq / 4);
  while (!motor_y.control_vars->ref_switch->is_pressed) {
    ESP_LOGI("calibration_initialization_task", "calibrating motor_y");
    vTaskDelay(20);
  }
  motor_set_target_frequency(&motor_y, 0);
  encoder_zero_position(motor_y.control_vars->ref_encoder);
  motor_y.control_vars->encoder_target_pos = 0;

  vTaskDelay(2000 / portTICK_PERIOD_MS);

  while (!motor_z.control_vars->ref_switch->is_pressed) {
    motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq / 4);
    ESP_LOGI("calibration_initialization_task", "calibrating motor_z");
    vTaskDelay(20);
  }

  ESP_LOGI("calibration_initialization_task", "finished motor_z cal");
  motor_set_target_frequency(&motor_z, 0);
  encoder_zero_position(motor_z.control_vars->ref_encoder);
  motor_z.control_vars->encoder_target_pos = 0;
  motor_y.control_vars->encoder_target_pos = 0;

  /* motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq); */
  /* while (!motor_y.control_vars->ref_switch->is_pressed) { */
  /*   ESP_LOGI("calibration_initialization_task", "calibrating motor_z"); */
  /*   vTaskDelay(20); */
  /* } */
  /* motor_set_target_frequency(&motor_z, 0); */
  /* encoder_zero_position(motor_z.control_vars->ref_encoder); */
  /* motor_z.control_vars->encoder_target_pos = 0; */

  while (1) {
    vTaskDelay(5000);
  }

  /* // go to middle */
  /* motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq / 4); */
  /* while (!motor_z.control_vars->ref_switch->is_pressed) { */
  /*   ESP_LOGI("calibration_initialization_task", "calibrating motor_z"); */
  /*   vTaskDelay(20); */
  /* } */
  // set new value

  return;
}

//////////////////////////////////
////// Call functions ////////////
//////////////////////////////////

void loop_scara_task() {
  while (1) {
    ESP_LOGI("loop_scara_task", "dir 1");
    /* gpio_set_level(motor_x.gpio_dir, 1); */
    /* gpio_set_level(motor_x.gpio_stp, 1); */
    /**/
    /* gpio_set_level(motor_y.gpio_dir, 1); */
    /* gpio_set_level(motor_y.gpio_stp, 1); */
    /**/
    /* gpio_set_level(motor_z.gpio_dir, 1); */
    /* gpio_set_level(motor_z.gpio_stp, 1); */

    /* gpio_set_level(motor_a.gpio_dir, 1); */
    /* gpio_set_level(motor_a.gpio_stp, 1); */

    /* gpio_set_level(motor_b.gpio_stp, 1); */
    /* gpio_set_level(motor_b.gpio_dir, 1); */

    /* gpio_set_level(motor_z.gpio_dir, 1); */
    /* gpio_set_level(motor_a.gpio_dir, 1); */
    /* gpio_set_level(motor_b.gpio_dir, 1); */

    gpio_set_level(motor_x.gpio_dir, 1);
    gpio_set_level(motor_y.gpio_dir, 1);
    gpio_set_level(motor_z.gpio_dir, 1);
    gpio_set_level(motor_a.gpio_dir, 1);
    gpio_set_level(motor_b.gpio_dir, 1);

    motor_set_target_frequency(&motor_x, motor_x.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_y, motor_y.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_a, motor_a.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_b, motor_b.pwm_vars->max_freq);
    /* motor_move_steps(&motor_x, -4000, motor_x.pwm_vars->max_freq); */
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    motor_set_target_frequency(&motor_x, 0);
    motor_set_target_frequency(&motor_y, 0);
    motor_set_target_frequency(&motor_z, 0);
    motor_set_target_frequency(&motor_a, 0);
    motor_set_target_frequency(&motor_b, 0);
    /* motor_move_steps(&motor_x, 4000, motor_x.pwm_vars->max_freq); */
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI("loop_scara_task", "dir 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    gpio_set_level(motor_y.gpio_dir, 0);
    gpio_set_level(motor_z.gpio_dir, 0);
    gpio_set_level(motor_a.gpio_dir, 0);
    gpio_set_level(motor_b.gpio_dir, 0);
    /* motor_move_steps(&motor_x, -1000, motor_x.pwm_vars->max_freq); */

    motor_set_target_frequency(&motor_x, motor_x.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_y, motor_y.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_a, motor_a.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_b, motor_b.pwm_vars->max_freq);
    /* motor_move_steps(&motor_x, 1000, motor_x.pwm_vars->max_freq); */
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    motor_set_target_frequency(&motor_x, 0);
    motor_set_target_frequency(&motor_y, 0);
    motor_set_target_frequency(&motor_z, 0);
    motor_set_target_frequency(&motor_a, 0);
    motor_set_target_frequency(&motor_b, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

#include "loadcell.h"

// Configuration for HX711 sensor 1
hx711_config_t hx711_config_1 = {
    .data_pin = HX711_1_DATA_PIN,
    .clock_pin = HX711_1_CLOCK_PIN,
    .gain = HX711_GAIN_64,
    .offset = 0,
    .scale = 1.0f,
    .sensor_id = HX711_SENSOR_1,
};

// Configuration for HX711 sensor 2
hx711_config_t hx711_config_2 = {
    .data_pin = HX711_2_DATA_PIN,
    .clock_pin = HX711_2_CLOCK_PIN,
    .gain = HX711_GAIN_64,
    .offset = 0,
    .scale = 1.0f,
    .sensor_id = HX711_SENSOR_2,
};

void loop_scara_readings() {
  while (1) {
    ESP_LOGI("switch_0", "is pressed: %d", switch_0.is_pressed);
    ESP_LOGI("switch_1", "is pressed: %d", switch_1.is_pressed);

    ESP_LOGI(encoder_0.label, "value: %f", encoder_0.angle_degrees);
    ESP_LOGI(encoder_1.label, "value: %f", encoder_1.angle_degrees);
    ESP_LOGI(encoder_2.label, "value: %f", encoder_2.angle_degrees);
    ESP_LOGI(encoder_3.label, "value: %f", encoder_3.angle_degrees);

    ESP_LOGI(encoder_0.label, "value: %d", encoder_0.accumulated_steps);
    ESP_LOGI(encoder_1.label, "value: %d", encoder_1.accumulated_steps);
    ESP_LOGI(encoder_2.label, "value: %d", encoder_2.accumulated_steps);
    ESP_LOGI(encoder_3.label, "value: %d", encoder_3.accumulated_steps);

    // Temporary test - read raw values directly
    int32_t test_raw1 = hx711_read_raw(&hx711_config_1);
    int32_t test_raw2 = hx711_read_raw(&hx711_config_2);
    ESP_LOGI(TAG, "Raw after tare - Sensor 1: %ld, Sensor 2: %ld", test_raw1,
             test_raw2);

    ESP_LOGI("", "");
    /* ESP_LOGI("loop_scara_readings", "motor freq: %f", */
    /* motor_b.pwm_vars->current_freq_hz); */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return;
}

void calibration_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting calibration process for both sensors...");

  // Wait a bit for system to stabilize
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // Step 1: Check initial raw readings
  ESP_LOGI(TAG, "=== INITIAL RAW READINGS ===");
  /* int32_t initial_raw1 = hx711_read_raw(&hx711_config_1); */
  int32_t initial_raw2 = hx711_read_raw(&hx711_config_2);
  /* ESP_LOGI(TAG, "Sensor 1 initial raw: %ld", initial_raw1); */
  ESP_LOGI(TAG, "Sensor 2 initial raw: %ld", initial_raw2);

  // Step 2: Tare both scales
  ESP_LOGI(TAG, "Please ensure no weight is on either scale");
  ESP_LOGI(TAG, "Taring both sensors in 5 seconds...");
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  // Reset scale to 1.0 before taring (important!)
  /* hx711_config_1.scale = 1.0f; */
  hx711_config_2.scale = 1.0f;

  ESP_LOGI(TAG, "=== BEFORE TARING ===");
  /* ESP_LOGI(TAG, "Sensor 1 - Offset: %ld, Scale: %.6f", hx711_config_1.offset, */
  /*          hx711_config_1.scale); */
  ESP_LOGI(TAG, "Sensor 2 - Offset: %ld, Scale: %.6f", hx711_config_2.offset,
           hx711_config_2.scale);

  hx711_tare(&hx711_config_1, 10); // Average of 10 readings
  /* hx711_tare(&hx711_config_2, 10); // Average of 10 readings */

  ESP_LOGI(TAG, "=== AFTER TARING ===");
  /* ESP_LOGI(TAG, "Sensor 1 - Offset: %ld, Scale: %.6f", hx711_config_1.offset, */
  /*          hx711_config_1.scale); */
  ESP_LOGI(TAG, "Sensor 2 - Offset: %ld, Scale: %.6f", hx711_config_2.offset,
           hx711_config_2.scale);

  // Step 3: Verify taring by reading raw values multiple times
  ESP_LOGI(TAG, "=== VERIFYING TARE (5 readings each) ===");
  for (int i = 0; i < 5; i++) {
    /* int32_t raw1 = hx711_read_raw(&hx711_config_1); */
    int32_t raw2 = hx711_read_raw(&hx711_config_2);

    // Calculate what the weight should be manually
    /* float weight1 = (raw1 - hx711_config_1.offset) / hx711_config_1.scale; */
    float weight2 = (raw2 - hx711_config_2.offset) / hx711_config_2.scale;

    ESP_LOGI(TAG, "Reading %d:", i + 1);
    /* ESP_LOGI(TAG, "  Sensor 1 - Raw: %ld, Calculated weight: %.2f g", raw1, */
    /*          weight1); */
    ESP_LOGI(TAG, "  Sensor 2 - Raw: %ld, Calculated weight: %.2f g", raw2,
             weight2);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  // Step 4: Check if the problem is in the global data structure update
  ESP_LOGI(TAG, "=== CHECKING GLOBAL DATA STRUCTURE ===");
  ESP_LOGI(TAG, "Waiting 2 seconds for data to update...");
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  /* if (g_hx711_data[HX711_SENSOR_1].data_ready) { */
  /*   ESP_LOGI(TAG, "Sensor 1 global data:"); */
  /*   ESP_LOGI(TAG, "  Raw: %ld", g_hx711_data[HX711_SENSOR_1].raw_value); */
  /*   ESP_LOGI(TAG, "  Weight: %.2f g", */
  /*            g_hx711_data[HX711_SENSOR_1].weight_grams); */
  /*   ESP_LOGI(TAG, "  Expected raw after offset: %ld", */
  /*            g_hx711_data[HX711_SENSOR_1].raw_value - hx711_config_1.offset); */
  /* } */

  if (g_hx711_data[HX711_SENSOR_2].data_ready) {
    ESP_LOGI(TAG, "Sensor 2 global data:");
    ESP_LOGI(TAG, "  Raw: %ld", g_hx711_data[HX711_SENSOR_2].raw_value);
    ESP_LOGI(TAG, "  Weight: %.2f g",
             g_hx711_data[HX711_SENSOR_2].weight_grams);
    ESP_LOGI(TAG, "  Expected raw after offset: %ld",
             g_hx711_data[HX711_SENSOR_2].raw_value - hx711_config_2.offset);
  }

  // Only continue with calibration if taring worked
  ESP_LOGI(TAG, "=== TARING ANALYSIS ===");
  /* int32_t final_raw1 = hx711_read_raw(&hx711_config_1); */
  int32_t final_raw2 = hx711_read_raw(&hx711_config_2);
  /* int32_t adjusted_raw1 = final_raw1 - hx711_config_1.offset; */
  int32_t adjusted_raw2 = final_raw2 - hx711_config_2.offset;

  /* ESP_LOGI(TAG, "Sensor 1: Raw=%ld, Offset=%ld, Adjusted=%ld", final_raw1, */
  /*          hx711_config_1.offset, adjusted_raw1); */
  ESP_LOGI(TAG, "Sensor 2: Raw=%ld, Offset=%ld, Adjusted=%ld", final_raw2,
           hx711_config_2.offset, adjusted_raw2);

  /* if (abs(adjusted_raw1) > 1000 || abs(adjusted_raw2) > 1000) { */
  /*   ESP_LOGE(TAG, */
  /*            "TARING FAILED! Adjusted raw values should be close to zero."); */
  /*   ESP_LOGE(TAG, */
  /*            "This indicates a problem with the HX711 library or hardware."); */
  /*   ESP_LOGE(TAG, "Debug suggestions:"); */
  /*   ESP_LOGE(TAG, */
  /*            "1. Check if hx711_tare() actually updates the offset correctly"); */
  /*   ESP_LOGE(TAG, */
  /*            "2. Check if the offset is being used in weight calculations"); */
  /*   ESP_LOGE(TAG, "3. Verify hardware connections are stable"); */
  /*   ESP_LOGE(TAG, "4. Try power cycling the load cells"); */

    // Don't continue with calibration if taring failed
    /* ESP_LOGE(TAG, "Stopping calibration process due to taring failure"); */
    /* vTaskDelete(NULL); */
  /*   return; */
  /* } */

  ESP_LOGI(TAG, "Taring successful! Continuing with calibration...");

  // Step 5: Calibrate sensor 1
  /* ESP_LOGI(TAG, "Please place a known weight on SENSOR 1 ONLY"); */
  /* ESP_LOGI(TAG, "Calibrating sensor 1 in 20 seconds..."); */
  /* vTaskDelay(20000 / portTICK_PERIOD_MS); */

  /* float calibration_weight_1 = 58.0f; // 58 grams */
  /* hx711_calibrate(&hx711_config_1, calibration_weight_1, 10); */

  /* ESP_LOGI(TAG, "Remove weight from sensor 1"); */
  /* vTaskDelay(3000 / portTICK_PERIOD_MS); */

  // Step 6: Calibrate sensor 2
  ESP_LOGW(TAG, "Please place a known weight on SENSOR 2 ONLY");
  ESP_LOGW(TAG, "Calibrating sensor 2 in 20 seconds...");
  vTaskDelay(20000 / portTICK_PERIOD_MS);

  float calibration_weight_2 = 241.0f;
  hx711_calibrate(&hx711_config_2, calibration_weight_2, 10);

  ESP_LOGI(TAG, "Calibration complete for both sensors!");
  /* ESP_LOGI(TAG, "Sensor 1 - Offset: %ld, Scale: %.6f", hx711_config_1.offset, */
  /*          hx711_config_1.scale); */
  ESP_LOGI(TAG, "Sensor 2 - Offset: %ld, Scale: %.6f", hx711_config_2.offset,
           hx711_config_2.scale);

  // Final test
  ESP_LOGI(TAG, "Final test - remove all weights and verify zero readings:");
  vTaskDelay(30000 / portTICK_PERIOD_MS);

  // Delete this task as calibration is done
  vTaskDelete(NULL);
}

void display_task(void *pvParameters) {
  ESP_LOGI(TAG, "Display task started");

  while (1) {
    ESP_LOGI(TAG, "=== HX711 Dual Sensor Readings ===");

    // Display sensor 1 data
    if (g_hx711_data[HX711_SENSOR_1].data_ready) {
      ESP_LOGI(TAG,
               "Sensor 1: %.2f g (%.3f kg) | Raw: %ld | Updated: %lu ms ago",
               g_hx711_data[HX711_SENSOR_1].weight_grams,
               g_hx711_data[HX711_SENSOR_1].weight_kg,
               g_hx711_data[HX711_SENSOR_1].raw_value,
               (esp_timer_get_time() / 1000) -
                   g_hx711_data[HX711_SENSOR_1].last_update_ms);
    } else {
      ESP_LOGW(TAG, "Sensor 1: No data available");
    }

    // Display sensor 2 data
    if (g_hx711_data[HX711_SENSOR_2].data_ready) {
      ESP_LOGI(TAG,
               "Sensor 2: %.2f g (%.3f kg) | Raw: %ld | Updated: %lu ms ago",
               g_hx711_data[HX711_SENSOR_2].weight_grams,
               g_hx711_data[HX711_SENSOR_2].weight_kg,
               g_hx711_data[HX711_SENSOR_2].raw_value,
               (esp_timer_get_time() / 1000) -
                   g_hx711_data[HX711_SENSOR_2].last_update_ms);
    } else {
      ESP_LOGW(TAG, "Sensor 2: No data available");
    }

    // Calculate total weight if both sensors have data
    if (g_hx711_data[HX711_SENSOR_1].data_ready &&
        g_hx711_data[HX711_SENSOR_2].data_ready) {
      float total_weight_g = g_hx711_data[HX711_SENSOR_1].weight_grams +
                             g_hx711_data[HX711_SENSOR_2].weight_grams;
      float total_weight_kg = total_weight_g / 1000.0f;
      ESP_LOGI(TAG, "Total Weight: %.2f g (%.3f kg)", total_weight_g,
               total_weight_kg);
    }

    ESP_LOGI(TAG, "================================");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void force_sensor_initialization_func() {
  ESP_LOGI(TAG, "Dual HX711 Force Sensor Application Starting...");

  // Initialize HX711 sensor 1
  esp_err_t ret1 = hx711_init(&hx711_config_1);
  if (ret1 != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize HX711 sensor 1: %s",
             esp_err_to_name(ret1));
    ESP_LOGE(TAG, "Please check sensor 1 connections:");
    ESP_LOGE(TAG, "1. HX711 VDD connected to 3.3V or 5V");
    ESP_LOGE(TAG, "2. HX711 GND connected to ESP32 GND");
    ESP_LOGE(TAG, "3. HX711 DT connected to GPIO %d", HX711_1_DATA_PIN);
    ESP_LOGE(TAG, "4. HX711 SCK connected to GPIO %d", HX711_1_CLOCK_PIN);
    ESP_LOGE(TAG, "5. Load cell properly connected to HX711");
    return;
  }

  // Initialize HX711 sensor 2
  esp_err_t ret2 = hx711_init(&hx711_config_2);
  if (ret2 != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize HX711 sensor 2: %s",
             esp_err_to_name(ret2));
    ESP_LOGE(TAG, "Please check sensor 2 connections:");
    ESP_LOGE(TAG, "1. HX711 VDD connected to 3.3V or 5V");
    ESP_LOGE(TAG, "2. HX711 GND connected to ESP32 GND");
    ESP_LOGE(TAG, "3. HX711 DT connected to GPIO %d", HX711_2_DATA_PIN);
    ESP_LOGE(TAG, "4. HX711 SCK connected to GPIO %d", HX711_2_CLOCK_PIN);
    ESP_LOGE(TAG, "5. Load cell properly connected to HX711");
    return;
  }

  ESP_LOGI(TAG, "Both HX711 sensors initialized successfully!");

  // Create calibration task (runs once)
  xTaskCreate(calibration_task, "calibration_task", 4096, NULL, 5, NULL);

  // Create HX711 reading tasks for both sensors
  xTaskCreate(hx711_task, "hx711_task_1", 4096, &hx711_config_1, 6, NULL);
  xTaskCreate(hx711_task, "hx711_task_2", 4096, &hx711_config_2, 6, NULL);

  // Create display task
  xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);

  ESP_LOGI(TAG, "All tasks created successfully");

  while (1) {
    // Example: Access global data structures for both sensors
    if (g_hx711_data[HX711_SENSOR_1].data_ready &&
        g_hx711_data[HX711_SENSOR_2].data_ready) {
      // You can use both sensors' data from anywhere in your application
      // g_hx711_data[HX711_SENSOR_1].weight_grams,
      // g_hx711_data[HX711_SENSOR_1].weight_kg
      // g_hx711_data[HX711_SENSOR_2].weight_grams,
      // g_hx711_data[HX711_SENSOR_2].weight_kg
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void init_scara() {
  force_sensor_initialization_func();
  /* wifi_initialization_func(); */
  /* switch_initialization_task(); */
  /* encoder_initialization_task(); */
  /* motor_initialization_task(); */
  /* xTaskCreate(loop_scara_task, "testloop", 4096, NULL, 5, NULL); */
  xTaskCreate(loop_scara_readings, "testreadings", 4096, NULL, 5, NULL);
  /* calibration_initialization_task(); */
  ESP_LOGI(TAG, "init_scara completed");
}
