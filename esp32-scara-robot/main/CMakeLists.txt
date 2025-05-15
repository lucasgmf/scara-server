idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES esp_driver_mcpwm
                       INCLUDE_DIRS "."
                       REQUIRES esp_timer driver
                   )
