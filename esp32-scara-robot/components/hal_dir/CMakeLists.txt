idf_component_register(SRCS "encoder.c" "motor.c" "switch_h.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_timer driver drivers devices freertos)
