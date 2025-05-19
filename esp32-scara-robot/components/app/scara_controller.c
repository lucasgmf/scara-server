#include "scara_controller.h"

static const char *TAG = "scara_controller";

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

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle1;

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = TEST_I2C_PORT,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

static i2c_device_config_t dev_cfg1 = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AS5600_I2C_ADDR,
    .scl_speed_hz = 100000,
};

static SemaphoreHandle_t i2c_mutex;
static encoder_conf encoder1 = {
    .i2c_mst_config = &i2c_mst_config,
    .bus_handle = &bus_handle,
    .dev_cfg = &dev_cfg1,
    .as5600_dev_handle = &dev_handle1,
    .reg_addr = AS5600_REG_ANGLE_MSB,
    .label = "Encoder 1",
    .i2c_mutex = &i2c_mutex,
    .i2c_timeout_ticks = portMAX_DELAY,
};

void init_scara() {
  i2c_mutex = xSemaphoreCreateMutex();
  init_encoder(&encoder1);

  ESP_LOGI(TAG, "");
  return;
}

void loop_scara() {
  xTaskCreate(encoder_task, "encoder1_task", 4096, &encoder1, 5, NULL);
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return;
}
