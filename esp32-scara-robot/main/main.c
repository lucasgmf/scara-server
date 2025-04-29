#include "ang_sensor.h"
#include "motor.h"
#include "motor_task.h"

void app_main(void) {

  init_i2c_master();

  while (1) {
    read_as5600_angle();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return;
}
