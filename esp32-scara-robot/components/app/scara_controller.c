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

#define I2C_ADDR_TCA 0x70
#define I2C_ADDR_AS5600 0x36

#define GPIO_I2C_SCL GPIO_NUM_22
#define GPIO_I2C_SDA GPIO_NUM_21

#define I2C_PORT I2C_NUM_0

#define I2C_MASTER_GLITCH_IGNORE_CNT 7
#define I2C_ENABLE_INTERNAL_PULLUP true

// ensure only one task access shared i2c bus
static SemaphoreHandle_t i2c_mutex;

///
/////////////// master bus ///////////////
///

// represents initialized i2c master bus
static i2c_master_bus_handle_t master_bus_handle;

static i2c_master_bus_config_t master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .sda_io_num = GPIO_I2C_SDA,
    .scl_io_num = GPIO_I2C_SCL,
    .glitch_ignore_cnt = I2C_MASTER_GLITCH_IGNORE_CNT,
    .flags.enable_internal_pullup = I2C_ENABLE_INTERNAL_PULLUP,
};

typedef struct {
  i2c_master_bus_config_t *bus_cfg;
  i2c_master_bus_handle_t *bus_handle;
  SemaphoreHandle_t *i2c_mutex;
  TickType_t timeout_ticks;
} i2c_master_config_t;

static i2c_master_config_t i2c_master_conf = {
    .bus_cfg = &master_bus_config,
    .bus_handle = &master_bus_handle,
    .i2c_mutex = &i2c_mutex,
    .timeout_ticks = portMAX_DELAY,
};

///
/////////////// slave bus ///////////////
///

#define I2C_DEVICE_SPEED_HZ 100000
// represents each device on the bus
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

typedef struct {
  i2c_device_config_t *dev_cfg;
  i2c_master_dev_handle_t *dev_handle;
} i2c_slave_bus_params;

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

typedef struct {
  const char *label;
  i2c_master_config_t *i2c_master;
  i2c_slave_bus_params *i2c_slave;
  i2c_slave_bus_params *i2c_tca;
  uint8_t tca_channel;
  uint8_t reg_angle_msb;
  uint16_t reg_angle_mask;
  // additional ...
  int offset;   // offset from 0
  bool reverse; // TODO: change this later
} encoder_t;

#define ENCODER_MSB_ANGLE_REG 0x0E
#define ENCODER_ANGLE_MASK 0x0FFF

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
};

esp_err_t tca_select_channel(uint8_t channel,
                             i2c_master_dev_handle_t *tca_handle) {
  if (channel > 7)
    return ESP_ERR_INVALID_ARG;

  uint8_t data = 1 << channel;
  return i2c_master_transmit(*tca_handle, &data, 1, portMAX_DELAY);
}

esp_err_t encoder_init(encoder_t *encoder) {
  if (!encoder || !encoder->i2c_master || !encoder->i2c_slave) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!encoder->i2c_master->bus_handle || !encoder->i2c_slave->dev_cfg ||
      !encoder->i2c_slave->dev_handle) {
    return ESP_ERR_INVALID_STATE;
  }

  // Add encoder device to the bus
  return i2c_master_bus_add_device(*encoder->i2c_master->bus_handle,
                                   encoder->i2c_slave->dev_cfg,
                                   encoder->i2c_slave->dev_handle);
}

uint16_t encoder_read_angle(encoder_t *encoder) {
  uint8_t reg = encoder->reg_angle_msb;
  uint8_t data[2];

  if (xSemaphoreTake(*encoder->i2c_master->i2c_mutex,
                     encoder->i2c_master->timeout_ticks) != pdTRUE) {
    ESP_LOGE(encoder->label, "Mutex timeout");
    return 0xFFFF;
  }

  esp_err_t ret =
      tca_select_channel(encoder->tca_channel, encoder->i2c_tca->dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(encoder->label, "TCA channel select failed: %s",
             esp_err_to_name(ret));
    xSemaphoreGive(*encoder->i2c_master->i2c_mutex);
    return 0xFFFF;
  }

  ret =
      i2c_master_transmit_receive(*encoder->i2c_slave->dev_handle, &reg, 1,
                                  data, 2, encoder->i2c_master->timeout_ticks);

  xSemaphoreGive(*encoder->i2c_master->i2c_mutex);

  if (ret != ESP_OK) {
    ESP_LOGE(encoder->label, "Read error: %s", esp_err_to_name(ret));
    return 0xFFFF;
  }

  uint16_t raw = ((data[0] << 8) | data[1]) & encoder->reg_angle_mask;
  if (encoder->reverse)
    raw = encoder->reg_angle_mask - raw;
  raw = (raw + encoder->offset) & encoder->reg_angle_mask;

  return raw;
}

void encoder_task(void *arg) {
  encoder_t *encoder = (encoder_t *)arg;
  if (encoder == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    uint16_t angle = encoder_read_angle(encoder);

    if (angle != 0xFFFF) {
      ESP_LOGI(encoder->label, "Angle: %u", angle);
    } else {
      ESP_LOGW(encoder->label, "Failed to read angle");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

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

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_1.tca_channel, encoder_1.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_1.dev_cfg,
      i2c_slave_conf_encoder_1.dev_handle));

  ESP_ERROR_CHECK(encoder_init(&encoder_1));

  xTaskCreate(encoder_task, "encoder1_task", 4096, &encoder_1, 5, NULL);
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
    NULL,
    /* .motor = &motor_x, */
    .pid = &pid1,
    .target_position = 1200.0f,
    .max_output_freq_hz = 1100.0f,
    .direction_reversed = false,
};

void motor_initialization_task() {
  motor_init_dir(&motor_x);
  motor_create_pwm(&motor_x);

  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &loop1, 5, NULL);
  return;
}

void init_scara() {
  /* wifi_initialization_func(); */
  encoder_initialization_task();
  /* motor_initialization_task(); */

  ESP_LOGI(TAG, "");
  return;
}

void loop_scara() {
  while (1) {
    /* ESP_LOGW(TAG, "changing target_position to %d", */
    /*          esp_net_conf.rec_data->target_position_1); */
    /* loop1.target_position = esp_net_conf.rec_data->target_position_1; */
    /* vTaskDelay(5000 / portTICK_PERIOD_MS); */
    /**/
    /* ESP_LOGW(TAG, "changing target_position to %d", */
    /*          esp_net_conf.rec_data->target_position_2); */
    /* loop1.target_position = esp_net_conf.rec_data->target_position_2; */
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

  return;
}
