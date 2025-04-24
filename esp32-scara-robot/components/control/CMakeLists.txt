idf_component_register(SRCS "motor_task.c" "pid.c"
                       INCLUDE_DIRS "."
                       REQUIRES hal freertos driver hal_user)
