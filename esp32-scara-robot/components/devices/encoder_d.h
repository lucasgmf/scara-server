#ifndef ENCODER_D_H
#define ENCODER_D_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "encoder.h"
#include "switch_h.h" // calibrations ...

#define MAX_ENCODER_VAL 4096
#define DEGREES_PER_COUNT (360.0f / MAX_ENCODER_VAL) // change this

void check_encoder_cal(mag_encoder *encoder_n, uint16_t reading);

#endif // ENCODER_D_H
