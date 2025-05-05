#include "encoder.h"

uint16_t get_as5600_reading(i2c_master_dev_handle_t *as5600_dev_handle_t,
                            uint8_t msb_reg_angle) {
  uint8_t reg_addr = msb_reg_angle;
  uint8_t angle_data[2] = {0};

  esp_err_t ret = i2c_master_transmit_receive(*as5600_dev_handle_t, &reg_addr,
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
