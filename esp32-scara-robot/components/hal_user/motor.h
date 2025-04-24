#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLINK_LED 4

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
  bool gpiodir_val;
  bool gpiostp_val;
  int gpiostp;
  int gpiodir;
  int angle_deg; // TODO: Maybe float?
  motor_encoder *encoder;
} motor;

typedef struct {
  int gpio;
  bool value;
} micro_switch;

typedef struct {
  motor *motor_claw;
  micro_switch *switch_claw;
} claw;

typedef struct { // TODO: Update this struct
  int gpio;
} colision_detector;

typedef struct {
} scara;

void led_test(void);
void led_test_2(void);
void motor_test(motor *motor_n);
void driver_calibration(motor *motor_n);

#endif // MOTOR_H
