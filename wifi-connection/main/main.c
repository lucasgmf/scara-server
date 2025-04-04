#include "esp_log.h"
#include "nvs_flash.h"
#include "tcp_server.h"
#include "wifi.h"

void app_main(void) {
  esp_err_t status = WIFI_FAILURE;

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  status = connect_wifi();
  if (status != WIFI_SUCCESS) {
    ESP_LOGI("MAIN", "Failed to associate to AP, exiting...");
    return;
  }

  status = start_tcp_server();
  if (status != TCP_SUCCESS) {
    ESP_LOGI("MAIN", "Failed to start TCP server, exiting...");
    return;
  }
}
