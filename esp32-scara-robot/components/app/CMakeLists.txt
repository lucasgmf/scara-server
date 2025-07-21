idf_component_register(SRCS "scara_controller.c" "scara_tasks.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES wpa_supplicant
                       REQUIRES hal_dir drivers devices esp_timer network wifi_provisioning json esp_http_server)
