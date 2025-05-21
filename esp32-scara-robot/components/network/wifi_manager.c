#include "wifi_manager.h"

static const char *TAG = "TCP_SERVER";

#define WIFI_MAX_RETRY 3
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static void wifi_event_handler(void *handler_arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  network_configuration *net_conf = (network_configuration *)handler_arg;

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();

  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {

    if (net_conf->retry_num < WIFI_MAX_RETRY) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      esp_wifi_connect();
      net_conf->retry_num++;
      ESP_LOGI(TAG, "Retrying to connect to the AP (attempt %d)...",
               net_conf->retry_num);
    } else {
      xEventGroupSetBits(net_conf->s_wifi_event_group, WIFI_FAIL_BIT);
    }

  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    net_conf->retry_num = 0;
    xEventGroupSetBits(net_conf->s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

esp_err_t init_wifi(network_configuration *net_conf) {
  net_conf->s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  // Pass net_conf as the handler_arg
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      wifi_event_handler, net_conf, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      wifi_event_handler, net_conf, NULL);

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(ESP_IF_WIFI_STA, net_conf->wifi_config);
  esp_wifi_start();

  ESP_LOGI(TAG, "Wi-Fi initialization complete, waiting for connection...");

  EventBits_t bits = xEventGroupWaitBits(net_conf->s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to AP successfully");
    return ESP_OK;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGE(TAG, "Failed to connect to AP after retries");
    return ESP_FAIL;
  } else {
    ESP_LOGE(TAG, "Unexpected Wi-Fi event");
    return ESP_FAIL;
  }
}
