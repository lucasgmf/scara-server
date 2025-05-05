#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

#define MOTOR_TASK_PERIOD_MS 10 
static const char *TAG = "MotorTask";
