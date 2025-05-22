#include "i2c_bus.h"
#include "driver/i2c_master.h" // new driver API
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "i2c_scanner_ng";

void i2c_scanner_ng(void) {
  i2c_master_bus_config_t bus_cfg = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_MASTER_NUM,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle = NULL;

  esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
    return;
  }

  ESP_LOGI(TAG, "Starting I2C scanner...");

  for (uint8_t addr = 1; addr < 0x7F; addr++) {
    i2c_device_config_t dev_cfg = {
        .device_address = addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
      continue;
    }

    ret =
        i2c_master_probe(bus_handle, addr, 10); // Use bus_handle and addr here
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "Found device at 0x%02X", addr);
    }

    i2c_master_bus_rm_device(dev_handle); // Only pass dev_handle
  }

  i2c_del_master_bus(bus_handle);

  ESP_LOGI(TAG, "I2C scan done.");
}
