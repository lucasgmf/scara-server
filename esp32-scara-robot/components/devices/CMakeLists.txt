idf_component_register(SRCS "encoder_d.c" "motor_d.c" "loadcell.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver hal_dir control)
