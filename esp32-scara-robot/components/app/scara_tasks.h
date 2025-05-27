#ifndef SCARA_TASKS_H
#define SCARA_TASKS_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"
#include "motor_d.h"

#include "encoder.h"
#include "encoder_d.h"

#include "wifi_manager.h"

#include "switch_h.h"

void tcp_server_task(void *arg);
void encoder_task(void *arg);
void motor_control_task(void *arg);
void switch_task(void *arg);
void encoder_try_calibration_task(void *arg);

#endif // SCARA_TASKS_H
