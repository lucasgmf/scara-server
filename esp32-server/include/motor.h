#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLINK_LED 2

void motor_setup(void);

#endif // MOTOR_H
