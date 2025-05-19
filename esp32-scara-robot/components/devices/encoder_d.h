#ifndef ENCODER_D_H
#define ENCODER_D_H

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "encoder.h"

uint16_t encoder_read_angle(encoder_t *encoder);

#endif // ENCODER_D_H
