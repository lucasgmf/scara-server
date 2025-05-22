idf_component_register(SRCS "encoder.c" "motor.c" "switch_h.c" "i2c_multiplexer.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_timer driver drivers devices freertos)
