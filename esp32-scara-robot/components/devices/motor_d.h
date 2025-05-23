#ifndef MOTOR_D_H
#define MOTOR_D_H

#include "encoder.h"
#include "encoder_d.h"
#include "motor.h"
#include "pid.h"

typedef struct {
  /* encoder_t *encoder; */
  motor_t *motor;
  pid_controller_t *pid;
  float target_position;    // Desired encoder position
  float current_position;   // Read from encoder
  float output_freq_hz;     // Output to motor (used as PWM freq)
  float max_output_freq_hz; // Safety clamp
  bool direction_reversed;  // Optional motor direction flip
} motor_control_loop_t;

#endif // MOTOR_D_H
