#include "tcp_server.h"
#include "esp_log.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <string.h>

static const char *TAG = "TCP_SERVER";

esp_err_t start_tcp_server(void) {
  int server_sock, client_sock;
  struct sockaddr_in server_addr, client_addr;
  socklen_t client_len = sizeof(client_addr);
  char buffer[1024];

  server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (server_sock < 0) {
    ESP_LOGE(TAG, "Failed to create server socket.");
    return TCP_FAILURE;
  }

  ESP_LOGI(TAG, "Socket created successfully.");

  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(42424);

  if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) <
      0) {
    ESP_LOGE(TAG, "Failed to bind socket.");
    close(server_sock);
    return TCP_FAILURE;
  }

  ESP_LOGI(TAG, "Socket bound successfully.");

  if (listen(server_sock, 5) < 0) {
    ESP_LOGE(TAG, "Failed to listen on socket.");
    close(server_sock);
    return TCP_FAILURE;
  }

  ESP_LOGI(TAG, "Listening for incoming connections...");

  while (1) {
    client_sock =
        accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
    if (client_sock < 0) {
      ESP_LOGE(TAG, "Failed to accept client connection.");
      close(server_sock);
      return TCP_FAILURE;
    }

    ESP_LOGI(TAG, "Client connected from %s", inet_ntoa(client_addr.sin_addr));

    while (1) {
      memset(buffer, 0, sizeof(buffer));
      int bytes_received = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
      if (bytes_received <= 0) {
        ESP_LOGI(TAG, "Client disconnected.");
        close(client_sock);
        break;
      }

      buffer[bytes_received] = '\0';
      ESP_LOGI(TAG, "Received: %s", buffer);
    }
  }

  close(server_sock);
  return TCP_SUCCESS;
}
