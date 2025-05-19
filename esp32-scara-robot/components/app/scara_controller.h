#ifndef SCARA_CONTROLLER_H
#define SCARA_CONTROLLER_H

// app/scara_tasks
#include "scara_tasks.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

// hal_dir/switch_h
#include "switch_h.h"

// drivers/i2c_bus
#include "i2c_bus.h"
#define I2C_SCL GPIO_NUM_22
#define I2C_SDA GPIO_NUM_21
#define I2C_PORT I2C_NUM_0
#define AS5600_ADDR 0x36

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
