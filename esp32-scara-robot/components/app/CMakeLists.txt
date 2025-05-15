idf_component_register(SRCS "scara_controller.c" "scara_tasks.c"
                       INCLUDE_DIRS "."
                       REQUIRES hal_dir drivers devices esp_timer)
