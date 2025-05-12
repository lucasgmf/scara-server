#ifndef SCARA_CONTROLLER_H
#define SCARA_CONTROLLER_H

#include "driver/i2c_master.h"
#include "esp_log.h"

// drivers/i2c_bus
#include "i2c_bus.h"
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ 100000 // 100 ~ 400 kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define TEST_I2C_PORT 0

// hal_dir/encoder
#define AS5600_I2C_ADDR 0x36
#define AS5600_REG_ANGLE_MSB 0x0C
#define AS5600_REG_ANGLE_LSB 0x0D

// hal_dir/switch_h
#include "switch_h.h"

// hal_dir/encoder
#include "encoder.h"

// hal_dir/motor
#include "motor.h"
// motor ids
#define MOTOR_X 1
#define MOTOR_Y 2
#define MOTOR_Z 3
/* #define MOTOR_A 4 */

#define GPIOXDIR GPIO_NUM_16
#define GPIOYDIR GPIO_NUM_27
#define GPIOZDIR GPIO_NUM_14
/* #define GPIOADIR 50 // WARN - Find GPIO number */
#define GPIOXSTP GPIO_NUM_26
#define GPIOYSTP GPIO_NUM_25
#define GPIOZSTP GPIO_NUM_17
/* #define GPIOASTP 50 // WARN - Find GPIO number */

void init_scara();
void loop_scara();
#endif // SCARA_CONTROLLER_H
