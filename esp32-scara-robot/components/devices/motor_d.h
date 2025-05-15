#ifndef MOTOR_D_H
#define MOTOR_D_H

#include "motor.h"

#define MAX_ENCODER_VAL 4096
#define DEGREES_PER_COUNT (360.0f / MAX_ENCODER_VAL) // change this

void apply_motor_pwm(mcpwm_unit_t unit, mcpwm_timer_t timer, float duty_percent,
                     uint32_t freq_hz);
void check_motor_freq(motor_t *motor, int frequency_hz, float duty_cycle);
#endif // MOTOR_D_H
