#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "string.h"

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  int target_position_1;
  int target_position_2;
} wifi_rec_data;

typedef struct {
  wifi_config_t *wifi_config;
  unsigned short port;
  int retry_num;
  int rx_buffer_size;
  int addr_str_size;
  wifi_rec_data *rec_data;
  EventGroupHandle_t s_wifi_event_group;
} network_configuration;

esp_err_t init_wifi(network_configuration *net_conf);

#endif // WIFI_MANAGER_H
