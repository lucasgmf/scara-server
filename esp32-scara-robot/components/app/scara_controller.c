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

// TODO: FIX acceleration in the 0 transition!

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
    .Kp = 1,           // 1
    .Ki = 0.005 * 0.5, // 0.01
    .Kd = 0.2,         // 0.2
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

void motor_initialization_task() {
  motor_init_dir(&motor_x);
  motor_create_pwm(&motor_x);

  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_x, 5, NULL);
  return;
}

//////////////////////////////////
////// Call functions ////////////
//////////////////////////////////

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
