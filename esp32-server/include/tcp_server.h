#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "esp_err.h"

#define TCP_SUCCESS (1 << 0)
#define TCP_FAILURE (1 << 1)

esp_err_t start_tcp_server(void);

#endif // TCP_SERVER_H
