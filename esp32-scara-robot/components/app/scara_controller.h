#ifndef SCARA_CONTROLLER_H
#define SCARA_CONTROLLER_H

///////////////////////////////////////////////////////////////
/////////////////// HARDWARE ABSTRACT LAYER ///////////////////
///////////////////////////////////////////////////////////////

#include "i2c_multiplexer.h"
#define I2C_ADDR_TCA 0x70
#define I2C_ADDR_AS5600 0x36

#define GPIO_I2C_SCL GPIO_NUM_22
#define GPIO_I2C_SDA GPIO_NUM_21

#define I2C_PORT I2C_NUM_0
#define I2C_MASTER_GLITCH_IGNORE_CNT 7
#define I2C_ENABLE_INTERNAL_PULLUP true
#define I2C_DEVICE_SPEED_HZ 100000

#include "encoder.h"
#define ENCODER_MSB_ANGLE_REG 0x0E
#define ENCODER_ANGLE_MASK 0x0FFF

#include "motor.h"
#define MOTOR_X 0
#define MOTOR_Y 1
#define MOTOR_Z 2
#define MOTOR_A 3
#define MOTOR_B 4

#define GPIO_X_DIR GPIO_NUM_16
#define GPIO_Y_DIR GPIO_NUM_27
#define GPIO_Z_DIR GPIO_NUM_14
#define GPIO_A_DIR GPIO_NUM_18
#define GPIO_B_DIR GPIO_NUM_23

#define GPIO_X_STP GPIO_NUM_26
#define GPIO_Y_STP GPIO_NUM_25
#define GPIO_Z_STP GPIO_NUM_17
#define GPIO_A_STP GPIO_NUM_19
#define GPIO_B_STP GPIO_NUM_5

#define MOTOR_X_LABEL "Motor x"
#define MOTOR_X_ID 0
#define MOTOR_Y_LABEL "Motor y"
#define MOTOR_Y_ID 1
#define MOTOR_Z_LABEL "Motor z"
#define MOTOR_Z_ID 2
#define MOTOR_A_LABEL "Motor a"
#define MOTOR_A_ID 3
#define MOTOR_B_LABEL "Motor b"
#define MOTOR_B_ID 4

#define MCPWM_MAX_PERIOD_TICKS 60000 // WARN: maybe change this
#define MCPWM_MIN_PERIOD_TICKS 5
#define PWM_RESOLUTION_HZ 1000000

#include "motor_d.h"
#include "switch_h.h"


///////////////////////////////////////////////
/////////////////// DRIVERS ///////////////////
///////////////////////////////////////////////

#include "driver/i2c_master.h"

///////////////////////////////////////////////
/////////////////// DEVICES ///////////////////
///////////////////////////////////////////////

///////////////////////////////////////////////
/////////////////// CONTROL ///////////////////
///////////////////////////////////////////////

///////////////////////////////////////////////
///////////////////// APP /////////////////////
///////////////////////////////////////////////

#include "scara_tasks.h"

///////////////////////////////////////////////
/////////////////// NETWORK ///////////////////
///////////////////////////////////////////////

#include "wifi_manager.h"

#define PORT 42424
#define WIFI_SSID "lucas"
#define WIFI_PASS "dahyuntwice"

#define RX_BUFFER_SIZE 128
#define ADDR_STR_SIZE 128

void init_scara();
void loop_scara();

#endif // SCARA_CONTROLLER_H
