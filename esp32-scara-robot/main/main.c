#include "motor.h"

void app_main(void) {

  motor motor_x = {
      .angle_deg = 0,
      .gpiodir = GPIOXDIR,
      .gpiostp = GPIOXSTP,
  };
  /* driver_calibration(&motor_x); */
  motor_test(&motor_x);
}
