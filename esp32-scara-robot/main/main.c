#include "mag_encoder.h"
#include "micro_switch.h"
#include "motor.h"
#include "motor_task.h"

switch_t switch_1 = {
    .value = false,
    .gpio_port = GPIO_NUM_4,
};

mag_encoder encoder_1 = {
    .raw_val = 0,
    .offset = 0,
    .is_calibrated = false,
    .cal_switch = &switch_1,
};

void app_main(void) {
  // one-shot hardware initialization
  /* motor_init_all(); */
  switch_init(&switch_1);
  init_encoder_i2c_master();

  while (1) {

    vTaskDelay(1000 / portTICK_PERIOD_MS / 3);
  }
  return;
}
