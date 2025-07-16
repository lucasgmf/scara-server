idf_component_register(SRCS "wifi_manager.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_netif wifi_provisioning
                       REQUIRES esp_event esp_wifi nvs_flash wpa_supplicant)
