#include "ang_sensor.h"
#include "micro_switch.h"
#include "motor.h"
#include "motor_task.h"

motor_t motor_x = {
    .id = MOTOR_X,
    .gpio_dir = GPIOXDIR,
    .gpio_stp = GPIOXSTP,
    .delay_us = 600,
};

motor_t motor_y = {
    .id = MOTOR_Y,
    .gpio_dir = GPIOYDIR,
    .gpio_stp = GPIOYSTP,
    .delay_us = 600,
};

motor_t motor_z = {
    .id = MOTOR_Z,
    .gpio_dir = GPIOZDIR,
    .gpio_stp = GPIOZSTP,
    .delay_us = 500,
};

void app_main(void) {
  motor_initialization(&motor_x);
  motor_initialization(&motor_y);
  motor_initialization(&motor_z);

  /* motor_task_start(&motor_x, "task_x", 5, 1); */
  motor_task_start(&motor_y, "task_y", 5, 1);
  motor_task_start(&motor_z, "task_z", 5, 0);
  /* while (1) { */
  /*   vTaskDelay(1000 / portTICK_PERIOD_MS / 3); */
  /* } */
  /* return; */
}
