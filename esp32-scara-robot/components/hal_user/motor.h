#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_X 1
#define MOTOR_Y 2
#define MOTOR_Z 3
#define MOTOR_A 4

#define GPIOXDIR 16
#define GPIOYDIR 27
#define GPIOZDIR 14
#define GPIOADIR 50 // WARN - Find GPIO number
#define GPIOXSTP 26
#define GPIOYSTP 25
#define GPIOZSTP 17
#define GPIOASTP 50 // WARN - Find GPIO number

typedef struct {
  int gpio;
  int value; // TODO: idk?
} motor_encoder;

typedef struct {
  int id;
  gpio_num_t gpio_stp;
  gpio_num_t gpio_dir;
  int step_count;
  int target_steps;
  int speed;
  int period_ms;
} motor_t;

typedef struct {
  int gpio;
  bool value;
} micro_switch;

typedef struct {
  motor_t *motor_claw;
  micro_switch *switch_claw;
} claw;

typedef struct { // TODO: Update this struct
  int gpio;
} colision_detector;

typedef struct {
} scara;

void led_test(void);
void led_test_2(void);
void motor_test(motor_t *motor_n);
void driver_calibration(motor_t *motor_n);

#endif // MOTOR_H
