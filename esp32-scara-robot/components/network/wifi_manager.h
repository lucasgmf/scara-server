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

// TODO: change int to bool
typedef struct {
  int dir_kinematics_on;
  float d1;
  float theta2;
  float theta3;
  float theta4;
  float theta5;
  int inv_kinematics_on;
  float x;
  float y;
  float z;
  float w;
} user_input_data;

typedef struct {
  wifi_config_t *wifi_config;
  unsigned short port;
  int retry_num;
  int rx_buffer_size;
  int addr_str_size;
  wifi_rec_data *rec_data;
  EventGroupHandle_t s_wifi_event_group;
  user_input_data *user_input_data;
} network_configuration;

esp_err_t init_wifi(network_configuration *net_conf);

#endif // WIFI_MANAGER_H
