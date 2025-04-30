//? #pragma once
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "motor.h"

void motor_task(void *arg);
void motor_task_start(motor_t *motor, const char *task_name,
                      UBaseType_t priority, BaseType_t core_id);
#endif // MOTOR_TASK_H
