#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// motor ids
#define MOTOR_X 1
#define MOTOR_Y 2
#define MOTOR_Z 3
/* #define MOTOR_A 4 */

#define GPIOXDIR GPIO_NUM_16
#define GPIOYDIR GPIO_NUM_27
#define GPIOZDIR GPIO_NUM_14
/* #define GPIOADIR 50 // WARN - Find GPIO number */
#define GPIOXSTP GPIO_NUM_26
#define GPIOYSTP GPIO_NUM_25
#define GPIOZSTP GPIO_NUM_17
/* #define GPIOASTP 50 // WARN - Find GPIO number */

typedef struct {
  int id;
  gpio_num_t gpio_stp;
  gpio_num_t gpio_dir;
  int step_count;
  int target_steps;
  int speed; // TODO: does this makes sense?
  int period_ms;
  int delay_us;
} motor_t;

typedef struct {
  motor_t **motors;
  size_t count;
} motor_array_t;

void driver_calib(motor_t *motor_n);
void motor_init_all();
void motor_move(motor_t *motor_n, bool direction, int iteractions,
                int delay_us);

#endif // MOTOR_H
