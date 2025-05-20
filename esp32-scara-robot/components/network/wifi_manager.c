#include "wifi_manager.h"

static const char *TAG = "TCP_SERVER";

void init_wifi(network_configuration *net_conf) {
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(ESP_IF_WIFI_STA, net_conf->wifi_config);
  esp_wifi_start();
  esp_wifi_connect();
}
