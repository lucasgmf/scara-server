idf_component_register(SRCS "motor_task.c" "pid.c" "mag_enc_task.c"
                       INCLUDE_DIRS "."
                       REQUIRES hal freertos driver hal_user)
