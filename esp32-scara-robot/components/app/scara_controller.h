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

// hal_dir/motor
#include "motor.h"

// hal_dir/i2c_multiplexer
#include "i2c_multiplexer.h"

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

// network/wifi_manager
#include "wifi_manager.h"
#define PORT 42424
#define WIFI_SSID "lucas"
#define WIFI_PASS "dahyuntwice"

#define RX_BUFFER_SIZE 128
#define ADDR_STR_SIZE 128

void init_scara();
void loop_scara();

#endif // SCARA_CONTROLLER_H
