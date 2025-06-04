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

#define GPIO_SWITCH_0 GPIO_NUM_12
#define GPIO_SWITCH_1 GPIO_NUM_13

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
    .offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 706,
    .is_calibrated = false,
    .switch_n = &switch_0,
};

static encoder_t encoder_1 = {
    .label = "Encoder 1",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_1,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 1,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,
    .offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 706,
    .is_calibrated = false,
};

static encoder_t encoder_2 = {
    .label = "Encoder 2",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_2,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 2,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,
    .offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 706,
    .is_calibrated = false,
    /* .i2c_mutex = &i2c_mutex, */
    /* .switch_n = &switch_0, */
};

static encoder_t encoder_3 = {
    .label = "Encoder 3",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_3,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 3,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,
    .offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 706,
    .is_calibrated = false,
    /* .i2c_mutex = &i2c_mutex, */
    /* .switch_n = &switch_0, */
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
// TODO: FIX acceleration in the 0 transition!

#define MOTOR_X_LABEL "Motor x"
#define MOTOR_X_ID 0

#define MOTOR_Y_LABEL "Motor y"
#define MOTOR_Y_ID 0

#define MCPWM_MAX_PERIOD_TICKS 60000 // WARN: maybe change this
#define MCPWM_MIN_PERIOD_TICKS 5
#define PWM_RESOLUTION_HZ 1000000
/* #define PWM_MAX_FREQ_HZ 300 // 1100 */
/* #define PWM_MIN_FREQ_HZ 1 */
/* #define PWM_MAX_ACCEL_HZ 3000 */

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

motor_pwm_vars_t pwm_vars_x = {
    .step_count = 0,
    .max_freq = 250,
    .min_freq = 0,
    .max_accel = 1000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

motor_pwm_vars_t pwm_vars_y = {
    .step_count = 0,
    .max_freq = 200,
    .min_freq = 0,
    .max_accel = 1500,
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
    .Kp = 0.1, // 1
    .Ki = 0.0, // 0.01
    .Kd = 0.0, // 0.2
};

motor_control_vars control_vars_x = {
    /* .ref_encoder = &encoder_0, */
    .encoder_target_pos = 0,
    .enable_pid = true,
    .pid = &pid_motor_y,
    .ref_switch = &switch_0,
};

motor_control_vars control_vars_y = {
    .ref_encoder = &encoder_0,
    .encoder_target_pos = 0,
    .enable_pid = true,
    .pid = &pid_motor_y,
    .ref_switch = &switch_0,
};

motor_t motor_x = {
    .label = MOTOR_X_LABEL,
    .id = MOTOR_X_ID,
    .gpio_stp = GPIO_X_STP,
    .gpio_dir = GPIO_X_DIR,
    .mcpwm_vars = &mcpwm_vars_x,
    .pwm_vars = &pwm_vars_x,
    .control_vars = &control_vars_x,
};

motor_t motor_y = {
    .label = MOTOR_Y_LABEL,
    .id = MOTOR_Y_ID,
    .gpio_stp = GPIO_Y_STP,
    .gpio_dir = GPIO_Y_DIR,
    .mcpwm_vars = &mcpwm_vars_y,
    .pwm_vars = &pwm_vars_y,
    .control_vars = &control_vars_y,
};

void motor_initialization_task() {
  /* motor_init_dir(&motor_x); */
  /* motor_create_pwm(&motor_x); */

  motor_init_dir(&motor_y);
  motor_create_pwm(&motor_y);

  /* xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_x, 5, NULL); */
  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_y, 5, NULL);
  return;
}

//////////////////////////////////
////// Call functions ////////////
//////////////////////////////////

void init_scara() {
  /* wifi_initialization_func(); */
  /* switch_initialization_task(); */
  encoder_initialization_task();
  /* motor_initialization_task(); */

  ESP_LOGI(TAG, "");
  return;
}

void loop_scara() {
  bool testMotor_x = false;
  bool testMotor_y = false;

  while (1) {
    /* motor_x.control_vars->encoder_target_pos = 0 + 250; */
    /* vTaskDelay(10000 / portTICK_PERIOD_MS); */
    /* motor_x.control_vars->encoder_target_pos = 4096 - 250; */
    /* vTaskDelay(10000 / portTICK_PERIOD_MS); */
    ESP_LOGI(encoder_0.label, "value: %f", encoder_0.current_reading);
    ESP_LOGI(encoder_1.label, "value: %f", encoder_1.current_reading);
    ESP_LOGI(encoder_2.label, "value: %f", encoder_2.current_reading);
    ESP_LOGI(encoder_3.label, "value: %f", encoder_3.current_reading);
    /* motor_y.control_vars->encoder_target_pos = 0 + 500; */
    /* vTaskDelay(5000 / portTICK_PERIOD_MS); */
    /**/
    /* motor_y.control_vars->encoder_target_pos = 4096 - 500; */
    /* vTaskDelay(5000 / portTICK_PERIOD_MS); */

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* ESP_LOGI("switch_0", " value: %d", switch_0.is_pressed); */
    /* ESP_LOGI("switch_1", " value: %d", switch_1.is_pressed); */
    /* vTaskDelay(200 / portTICK_PERIOD_MS); */

    if (testMotor_x) {
      gpio_set_level(motor_x.gpio_dir, 1);
      motor_set_frequency(&motor_x, motor_x.pwm_vars->max_freq);
      vTaskDelay(2000 / portTICK_PERIOD_MS);

      motor_set_frequency(&motor_x, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      gpio_set_level(motor_x.gpio_dir, 0);
      motor_set_frequency(&motor_x, motor_x.pwm_vars->max_freq);
      vTaskDelay(2000 / portTICK_PERIOD_MS);

      motor_set_frequency(&motor_x, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (testMotor_y) {
      motor_y.control_vars->encoder_target_pos = 1000;
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      motor_y.control_vars->encoder_target_pos = 1500;
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    /* vTaskDelay(1000 / portTICK_PERIOD_MS); */
  }

  return;
}
