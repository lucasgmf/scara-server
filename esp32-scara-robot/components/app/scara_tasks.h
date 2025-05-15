#ifndef SCARA_TASKS_H
#define SCARA_TASKS_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"
#include "motor_d.h"

#include "encoder.h"
#include "encoder_d.h"

void accel_test_motor(void *arg);
void move_test_motor(void *arg);

#endif // SCARA_TASKS_H
