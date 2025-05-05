idf_component_register(SRCS "motor.c"
                            "micro_switch.c"
                            "mag_encoder.c"
                            "motor_driver.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver)
