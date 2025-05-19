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

void task_update_motor_pwm(void *arg);
void encoder_task(void *param);

#endif // SCARA_TASKS_H
