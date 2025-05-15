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

// drivers/i2c_bus
#include "i2c_bus.h"
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ 100000 // 100 ~ 400 kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define TEST_I2C_PORT 0

// drivers/motor_d
#include "motor_d.h"

// hal_dir/encoder
#define AS5600_I2C_ADDR 0x36
#define AS5600_REG_ANGLE_MSB 0x0C
#define AS5600_REG_ANGLE_LSB 0x0D

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t as5600_dev_handle;

void init_i2c_master() {
  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = TEST_I2C_PORT, // I2C_NUM_0
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = AS5600_I2C_ADDR,
      .scl_speed_hz = 100000,
  };

  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &dev_cfg, &as5600_dev_handle));
  ESP_LOGI("init_i2c_master", "init_i2c_master returned without errors");
  return;
}

uint16_t read_as5600_angle() {
  uint8_t reg_addr = AS5600_REG_ANGLE_MSB;
  uint8_t angle_data[2] = {0};

  esp_err_t ret = i2c_master_transmit_receive(as5600_dev_handle, &reg_addr,
                                              1, // Send register address
                                              angle_data, 2, // Read 2 bytes
                                              -1             // Blocking timeout
  );

  if (ret != ESP_OK) {
    printf("AS5600 read failed: %s\n", esp_err_to_name(ret));
    return 0xFFFF;
  }

  uint16_t angle = ((angle_data[0] << 8) | angle_data[1]) & 0x0FFF;
  ESP_LOGI("Main", "Reading... Value read: %u", angle);

  return ((angle_data[0] << 8) | angle_data[1]) & 0x0FFF; // 12-bit mask
}

void init_scara() {

  init_i2c_master();
  ESP_LOGI(TAG, "");
  return;
}

void loop_scara() {
  while (1) {
    read_as5600_angle();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  return;
}
