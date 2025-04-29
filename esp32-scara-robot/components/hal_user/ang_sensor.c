#include "ang_sensor.h"

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
  return ((angle_data[0] << 8) | angle_data[1]) & 0x0FFF; // 12-bit mask
}

uint16_t read_calibrated_angle(uint16_t offset) {
  uint16_t raw = read_as5600_angle();
  if (raw == 0xFFFF)
    return 0xFFFF;

  int32_t adjusted = raw - offset * 4096 / 360;
  if (adjusted < 0)
    adjusted += 4096;
  return adjusted;
}

#define MAX_ENCODER_VAL 4096

// Calibration: known encoder value at 0 degrees
#define ENCODER_AT_0_DEG 4025
#define DEGREES_PER_COUNT (180.0f / 6487.0f)

float encoder_to_degrees(int32_t unwrapped_val) {
  int32_t relative_position = unwrapped_val - ENCODER_AT_0_DEG;
  return relative_position * DEGREES_PER_COUNT;
}
