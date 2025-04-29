#ifndef ANG_SENSOR_H
#define ANG_SENSOR_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ 100000 // 100 ~ 400 kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define TEST_I2C_PORT 0

#define AS5600_I2C_ADDR 0x36
#define AS5600_REG_ANGLE_MSB 0x0C
#define AS5600_REG_ANGLE_LSB 0x0D

float encoder_to_degrees(int32_t unwrapped_val);
void init_i2c_master();
uint16_t read_as5600_angle();
uint16_t read_calibrated_angle(uint16_t offset);

#endif // ANG_SENSOR_H
