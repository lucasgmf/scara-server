#ifndef I2C_MULTIPLEXER_H
#define I2C_MULTIPLEXER_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

esp_err_t tca_select_channel(uint8_t channel,
                             i2c_master_dev_handle_t *tca_handle);

#endif // I2C_MULTIPLEXER_H
