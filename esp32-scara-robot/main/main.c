#include "motor.h"

void motor_x_task(void *params) { motor_test(GPIOXDIR, GPIOXSTP); }
void motor_y_task(void *params) { motor_test(GPIOYDIR, GPIOYSTP); }
void app_main(void) {
  xTaskCreate(motor_x_task, "Motor_x aka task1", 4096, NULL, 5, NULL);
  xTaskCreate(motor_y_task, "Motor_x aka task2", 4096, NULL, 5, NULL);
}
