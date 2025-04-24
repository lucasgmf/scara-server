
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

#define MOTOR_TASK_PERIOD_MS 1000

static const char *TAG = "MotorTask";

// TODO: watchdog, stats, or timing metrics
void motor_task(void *arg) {
  motor_t *motor = (motor_t *)arg;
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    /* motor_update(motor); */
    ESP_LOGI(TAG, "Hello from motor %d", motor->id);
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
  }
}

//
// --- 1. Update motors --- motor_test();

// --- 2. Monitor system state (optional) ---
// Example: check fault flags or encoder errors
// if (motor_fault_detected()) {
//     ESP_LOGE(TAG, "Motor fault detected!");
// }

// --- 3. Wait until next period ---
