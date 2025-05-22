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
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static i2c_master_bus_config_t bus_cfg = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .sda_io_num = I2C_SDA,
    .scl_io_num = I2C_SCL,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

static i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AS5600_ADDR,
    .scl_speed_hz = 100000,
};

static encoder_settings_t encoder1_settings = {
    .angle_reg = 0x0E,
    .angle_mask = 0x0FFF,
    .reverse = false,
    .zero_offset = 0,
};

static i2c_master_config_t i2c_master_conf = {
    .bus_cfg = &bus_cfg,
    .bus_handle = &bus_handle,
    .i2c_mutex = &i2c_mutex,
    .timeout_ticks = portMAX_DELAY,
};

static i2c_slave_config_t i2c_slave_conf = {
    .dev_cfg = &dev_cfg,
    .dev_handle = &dev_handle,
    .address = AS5600_ADDR,
};

static encoder_t encoder1 = {
    .settings = &encoder1_settings,
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf,
    .label = "Encoder 1",
};

void encoder_task(void *arg) {
  encoder_t *enc = (encoder_t *)arg;
  while (1) {
    encoder_read_angle(enc);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  return;
}

//////////////////////////////////
////// hal_dir/motor /////////////
//////////////////////////////////

motor_t motor_x = {
    .id = 0,
    .gpio_stp = GPIOXSTP,
    .gpio_dir = GPIOXDIR,
    .step_count = 0,
    .mcpwm_unit = 0,
    .mcpwm_timer = 0,
    .mcpwm_opr = 0,
    .move_ms = 0,
    .current_freq_hz = 200,
    .target_freq_hz = 0,
    .speed_hz = 800,
};

motor_t motor_y = {
    .id = 1,
    .gpio_stp = GPIOYSTP,
    .gpio_dir = GPIOYDIR,
    .step_count = 0,
    .mcpwm_unit = 1,
    .mcpwm_timer = 0,
    .mcpwm_opr = 0,
    .move_ms = 0,
    .current_freq_hz = 200,
    .target_freq_hz = 0,
    .speed_hz = 800,
};

static pid_controller_t pid1 = {
    .Kp = 1.2f,
    .Ki = 0.001f,
    .Kd = 0.1f,
    .output_limit = 500.0f,
};

static motor_control_loop_t loop1 = {
    /* .encoder = &encoder1, */
    .motor = &motor_x,
    .pid = &pid1,
    .target_position = 1200.0f,
    .max_output_freq_hz = 1100.0f,
    .direction_reversed = false,
};

void init_scara() {
  wifi_initialization_func();

  // PID + motor tasks
  i2c_mutex = xSemaphoreCreateMutex();
  motor_init_dir(&motor_x);
  motor_create_pwm(&motor_x);
  ESP_ERROR_CHECK(encoder_init(&encoder1));

  ESP_LOGI(TAG, "");
  return;
}

void loop_scara() {
  xTaskCreate(encoder_task, "encoder1_task", 4096, &encoder1, 5, NULL);
  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &loop1, 5, NULL);

  while (1) {
    ESP_LOGW(TAG, "changing target_position to %d",
             esp_net_conf.rec_data->target_position_1);
    loop1.target_position = esp_net_conf.rec_data->target_position_1;
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    ESP_LOGW(TAG, "changing target_position to %d",
             esp_net_conf.rec_data->target_position_2);
    loop1.target_position = esp_net_conf.rec_data->target_position_2;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

  return;
}
