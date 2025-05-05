#include "mag_encoder.h"
#include "micro_switch.h"
#include "motor.h"
#include "motor_task.h"

mag_encoder encoder_1 = {
    .raw_val = 0,
    .offset = 0,
    .is_calibrated = false,
};

switch_t switch_1 = {
    .value = false,
    .gpio_port = GPIO_NUM_4,
};

void app_main(void) {
  // one-shot hardware initialization
  /* motor_init_all(); */
  switch_init(&switch_1);
  init_encoder_i2c_master();

  /* temp */
  uint16_t angle = 0;

  while (1) {
    // routines
      update_encoder_val()




    angle = read_as5600_angle();
    update_encoder_val(&sens_1, angle);

    ESP_LOGI("Main",
             "\nraw value: %d\ncalibrated: %d\nencoded to degrees: %d\nsens_1 "
             "val: %d",
             angle, read_calibrated_angle(sens_1.offset),
             encoder_to_degrees(&sens_1), sens_1.raw_val);
    ESP_LOGI("Main", "\nSwitch value: %d", gpio_get_level(GPIO_NUM_4));

    vTaskDelay(1000 / portTICK_PERIOD_MS / 3);
  }
  return;
}
