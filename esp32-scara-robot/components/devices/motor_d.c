#include "motor_d.h"
#include "math.h"

void set_motor_for_calibration(motor_t *motor) {
  // slowly got to switch
  ESP_LOGI("set_motor_for_calibration", "will now check for switch value");

  while (motor->is_calibrated == false) {
    motor_set_frequency(motor, 200);
    gpio_set_level(motor->gpio_dir, 0);
    ESP_LOGI("set_motor_for_calibration", "moving motor");
    if (motor->control_vars->ref_switch->is_pressed == true) {
      motor->control_vars->ref_encoder->offset =
          motor->control_vars->ref_encoder->current_reading;
      ESP_LOGI("set_motor_for_calibration",
               "changing offset of encoder %c to %d",
               motor->control_vars->ref_encoder->label,
               motor->control_vars->ref_encoder->current_reading);
      break;
    }
  }
  motor_set_frequency(motor, 0);
  return;
}
