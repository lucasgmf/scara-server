#ifndef SCARA_TASKS_H
#define SCARA_TASKS_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder.h"
#include "encoder_d.h"

void update_encoder_val_task(void *arg);

#endif // SCARA_TASKS_H
