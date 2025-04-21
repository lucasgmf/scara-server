#include "motor.h"

void app_main(void) {
  /* led_test(); */
  /* led_test_2(); */
  motor_test(GPIOYDIR, GPIOYSTP);
  /* driver_calibration(GPIOYDIR, GPIOYSTP); */
}
