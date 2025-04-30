//? #pragma once
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "motor.h"

// TODO: watchdog, stats, or timing metrics
void motor_task(void *arg);
void motor_task_start(motor_t *motor, const char *task_name,
                      UBaseType_t priority, BaseType_t core_id);

// Using a task for each motor mas the moviments not fluid...

#endif // MOTOR_TASK_H
