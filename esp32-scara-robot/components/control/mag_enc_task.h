#ifndef MAG_ENC_TASK_H
#define MAG_ENC_TASK_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mag_encoder.h"

void update_encoder_val_task(void *arg);

#endif // MAG_ENC_TASK_H
