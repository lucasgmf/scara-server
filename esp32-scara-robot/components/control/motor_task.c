#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

#define MOTOR_TASK_PERIOD_MS 10 // TODO: Maybe decrese this to 5
static const char *TAG = "MotorTask";

void motor_control_task(void *arg) {
  motor_t *motor = (motor_t *)arg;
  if (motor == NULL) {
    ESP_LOGE(TAG, "parameter motor is null, aborting.");
    vTaskDelete(NULL);
    return;
  }
  TickType_t last_wake_time = xTaskGetTickCount();
  while (true) {
    // run motor adjust position or something
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
  }
}

/* void motors_task_init(motors_t *motors) {} */

void motor_task_init(motor_t *motor, const char *task_name,
                     UBaseType_t priority, BaseType_t core_id) {
  if (motor == NULL || task_name == NULL) {
    ESP_LOGE(TAG, "Cannot start motor task: invalid arguments");
    return;
  }

 /*  BaseType_t result = xTaskCreatePinnedToCore(motor_task, task_name, */
 /*                                              2048, // Stack size in words */
 /*                                              (void *)motor, priority, */
 /*                                              NULL, // No handle needed for now */
 /*                                              core_id); */
 /**/
 /*  if (result != pdPASS) { */
 /*    ESP_LOGE(TAG, "Failed to create task %s", task_name); */
 /*  } else { */
 /*    ESP_LOGI(TAG, "Motor task %s started on core %d", task_name, core_id); */
 /* } */
}
