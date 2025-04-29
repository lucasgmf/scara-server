#ifndef ANG_SENSOR_H
#define ANG_SENSOR_H

#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 0000    // Adjust for your pin
#define I2C_MASTER_SDA_IO 0000    // Adjust for your pin
#define I2C_MASTER_FREQ_HZ 100000 // 100kHz typical
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

void i2c_master_init(void);
esp_err_t i2c_scanner();
uint8_t read_byte_from_slave(uint8_t slave_addr, uint8_t reg_addr);

#endif // ANG_SENSOR_H
