#include "encoder_d.h"

int get_encoder_val_deg(mag_encoder *encoder_n) {
  int32_t adjusted_val = encoder_n->raw_val - encoder_n->offset;
  int angle_deg = (int)(adjusted_val * DEGREES_PER_COUNT);

  angle_deg = ((angle_deg % 360) + 360) % 360;
  return angle_deg;
}

void calibrate_encoder(uint32_t current_val, mag_encoder *encoder_n) {
  ESP_LOGI("calibrate_encoder", "Calibration has been successful");
  encoder_n->offset = current_val;
  return;
}

void check_encoder_cal(mag_encoder *encoder_n, uint16_t reading) {
  if (encoder_n->is_calibrated == false) {
    ESP_LOGW("check_encoder_cal", "Encoder with id %d is not calibrated!",
             encoder_n->id);
    // try calibration
    if (encoder_n->cal_switch->value == true) {
      ESP_LOGI("check_encoder_cal", "calibrating ...");
      calibrate_encoder(reading, encoder_n);
    }
  }
}
