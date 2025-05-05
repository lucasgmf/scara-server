#ifndef MAG_ENCODER_H
#define MAG_ENCODER_H

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

#define MAX_ENCODER_VAL 4096
#define DEGREES_PER_COUNT (360.0f / MAX_ENCODER_VAL) // change this

typedef struct {
  uint16_t raw_val;
  uint32_t offset;
  bool is_calibrated;
  int id;
} mag_encoder;

void init_encoder_i2c_master();
uint16_t get_as5600_reading();
int get_encoder_val_deg(mag_encoder *encoder_n);
void calibrate_encoder(uint32_t current_val, mag_encoder *sensor);

#endif // MAG_ENCODER_H
