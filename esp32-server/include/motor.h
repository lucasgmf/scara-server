#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLINK_LED 4

#define GPIOXDIR 16
#define GPIOYDIR 27
#define GPIOZDIR 14
#define GPIOXSTP 26
#define GPIOYSTP 25
#define GPIOZSTP 17

void led_test(void);
void motor_test(int gpiodir, int gpiostp);
void led_test_2(void);
void driver_calibration(int gpiodir, int gpiostp);

#endif // MOTOR_H
