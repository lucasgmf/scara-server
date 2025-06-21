#ifndef LOADCELL_H
#define LOADCELL_H
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// GPIO pin definitions for HX711 #1
#define HX711_1_DATA_PIN GPIO_NUM_4
#define HX711_1_CLOCK_PIN GPIO_NUM_2

// GPIO pin definitions for HX711 #2
#define HX711_2_DATA_PIN GPIO_NUM_13
#define HX711_2_CLOCK_PIN GPIO_NUM_12

// HX711 gain options
typedef enum {
  HX711_GAIN_128 = 1, // Channel A, gain 128
  HX711_GAIN_32 = 2,  // Channel B, gain 32
  HX711_GAIN_64 = 3   // Channel A, gain 64
} hx711_gain_t;

// HX711 sensor ID
typedef enum {
  HX711_SENSOR_1 = 0,
  HX711_SENSOR_2 = 1,
  HX711_SENSOR_COUNT = 2
} hx711_sensor_id_t;

// HX711 configuration structure
typedef struct {
  gpio_num_t data_pin;
  gpio_num_t clock_pin;
  hx711_gain_t gain;
  int32_t offset; // Tare value
  float scale;    // Scale factor for calibration
  hx711_sensor_id_t sensor_id;
} hx711_config_t;

// Global data structure for sensor readings
typedef struct {
  int32_t raw_value;
  float weight_grams;
  float weight_kg;
  bool data_ready;
  uint32_t last_update_ms;
} hx711_data_t;

// Global instances of sensor data for both sensors
extern hx711_data_t g_hx711_data[HX711_SENSOR_COUNT];

// Function prototypes
esp_err_t hx711_init(hx711_config_t *config);
bool hx711_is_ready(hx711_config_t *config);
int32_t hx711_read_raw(hx711_config_t *config);
int32_t hx711_read_average(hx711_config_t *config, uint8_t samples);
void hx711_tare(hx711_config_t *config, uint8_t samples);
void hx711_calibrate(hx711_config_t *config, float known_weight_grams,
                     uint8_t samples);
float hx711_get_weight(hx711_config_t *config);
void hx711_power_down(hx711_config_t *config);
void hx711_power_up(hx711_config_t *config);
void hx711_task(void *pvParameters);

#endif // LOADCELL_H
