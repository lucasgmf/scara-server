#include "ang-sensor.h"

// initialize driver
void i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                     I2C_MASTER_TX_BUF_DISABLE, 0);
}

// scan devices
esp_err_t i2c_scanner() {
  i2c_cmd_handle_t cmd;
  esp_err_t ret;
  for (int addr = 1; addr < 127; addr++) {
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
      printf("Found device at 0x%02X\n", addr);
    }
  }
  return ESP_OK;
}

uint8_t read_byte_from_slave(uint8_t slave_addr, uint8_t reg_addr) {
  uint8_t data = 0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // Write the register address
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);

  // Read from the register
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK) {
    return data;
  } else {
    printf("Error reading from device: %s\n", esp_err_to_name(ret));
    return 0xFF;
  }
}
