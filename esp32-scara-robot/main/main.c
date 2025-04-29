#include "ang_sensor.h"
#include "micro_switch.h"
#include "motor.h"
#include "motor_task.h"

void app_main(void) {

  button_init_func();
  while (1) {

    ESP_LOGI("main", "Value of the switch: %d", gpio_get_level(GPIO_NUM_4));
    vTaskDelay(1000 / portTICK_PERIOD_MS / 3);
  }
  return;
}
