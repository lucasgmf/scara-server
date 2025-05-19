#ifndef PID_H
#define PID_H

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prev_error;
  float output_limit; // Optional clamp
} pid_controller_t;

#endif // PID_H
