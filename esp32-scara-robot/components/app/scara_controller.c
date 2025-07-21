#include "scara_controller.h"

static const char *TAG = "scara_controller";

//////////////////////////////////
////// network/wifi_manager //////
//////////////////////////////////
///
///

#include "cJSON.h"
#include "esp_http_server.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

wifi_config_t wifi_config_a = {
    .sta =
        {
            .ssid = "eduroam",
            .threshold.authmode = WIFI_AUTH_WPA2_ENTERPRISE,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .failure_retry_cnt = 3,
        },
};

wifi_rec_data wifi_received_data = {
    .Kp = 1.2f,
    .Ki = 0.001f,
    .Kd = 0.1f,
    .target_position_1 = 2000,
    .target_position_2 = 2000,
};

user_input_data client_input_data = {
    .d1 = 1,
    .theta2 = 0,
    .theta3 = 0,
    .theta4 = 0,
    .x = 0,
    .y = 0,
    .z = 0,
    .w = 0,
    .dir_kinematics_on = 0,
    .inv_kinematics_on = 0,
    .pick_and_place_on = 0,
    .xip = 0,
    .yip = 0,
    .zip = 0,
    .wip = 0,
    .xfp = 0,
    .yfp = 0,
    .zfp = 0,
    .wfp = 0,
    .hp = 0,
};

system_output_data system_output_data_values = {
    .horizontal_load = 0.0f,
    .vertical_load = 0.0f,
    .encoder0 = 0.0f,
    .encoder1 = 0.0f,
    .encoder2 = 0.0f,
    .encoder3 = 0.0f,
    .switch0 = 0,
    .switch1 = 0,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f,
    .w = 0.0f,
    .d1 = 0.0f,
    .theta2 = 0.0f,
    .theta3 = 0.0f,
    .theta4 = 0.0f,
    .theta5 = 0.0f,
};

system_output_data system_new_output_data_values = {
    .horizontal_load = 0.0f,
    .vertical_load = 0.0f,
    .encoder0 = 0.0f,
    .encoder1 = 0.0f,
    .encoder2 = 0.0f,
    .encoder3 = 0.0f,
    .switch0 = 0,
    .switch1 = 0,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f,
    .w = 0.0f,
    .d1 = 0.0f,
    .theta2 = 0.0f,
    .theta3 = 0.0f,
    .theta4 = 0.0f,
    .theta5 = 0.0f,
};

network_configuration esp_net_conf = {
    .wifi_config = &wifi_config_a,
    .port = PORT,
    .retry_num = 0,
    .rx_buffer_size = RX_BUFFER_SIZE,
    .addr_str_size = ADDR_STR_SIZE,
    .rec_data = &wifi_received_data,
    .s_wifi_event_group = (EventGroupHandle_t)&wifi_received_data,
    .user_input_data = &client_input_data,
    .system_output_data = &system_output_data_values,
};

// Global variables
static int g_client_sock = -1;
static SemaphoreHandle_t g_client_sock_mutex = NULL;
static SemaphoreHandle_t g_system_output_mutex = NULL;
static bool g_client_connected = false;
static SemaphoreHandle_t g_connection_mutex = NULL;

// Helper function to safely set client socket and connection status
void set_client_socket(int sock) {
  if (xSemaphoreTake(g_client_sock_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    g_client_sock = sock;
    xSemaphoreGive(g_client_sock_mutex);
  }

  if (xSemaphoreTake(g_connection_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    g_client_connected = (sock >= 0);
    xSemaphoreGive(g_connection_mutex);
  }
}

// Helper function to safely get client socket
int get_client_socket() {
  int sock = -1;
  if (xSemaphoreTake(g_client_sock_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    sock = g_client_sock;
    xSemaphoreGive(g_client_sock_mutex);
  }
  return sock;
}

bool is_client_connected() {
  bool connected = false;
  if (xSemaphoreTake(g_connection_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    connected = g_client_connected;
    xSemaphoreGive(g_connection_mutex);
  }
  return connected;
}

bool handle_error(bool condition, const char *message, int sock_to_close) {
  if (condition) {
    ESP_LOGE(TAG, "%s: errno %d", message, errno);
    if (sock_to_close >= 0) {
      close(sock_to_close);
    }
    vTaskDelete(NULL);
    return true;
  }
  return false;
}
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"

static SemaphoreHandle_t g_user_input_mutex = NULL;

// HTML page stored in flash memory
static const char *html_page =
    "<!DOCTYPE html>"
    "<html><head>"
    "<title>ESP32 Control Panel</title>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<style>"
    "body { font-family: Arial; margin: 20px; background-color: #f0f0f0; }"
    ".container { max-width: 800px; margin: 0 auto; background: white; "
    "padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px "
    "rgba(0,0,0,0.1); }"
    ".section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; "
    "border-radius: 5px; }"
    ".section h3 { margin-top: 0; color: #333; }"
    ".input-group { margin: 10px 0; }"
    ".input-group label { display: inline-block; width: 100px; font-weight: "
    "bold; }"
    ".input-group input { width: 100px; padding: 5px; margin: 0 10px; }"
    ".checkbox-group { margin: 10px 0; }"
    ".checkbox-group input { margin-right: 10px; }"
    ".btn { background-color: #4CAF50; color: white; padding: 10px 20px; "
    "border: none; border-radius: 5px; cursor: pointer; margin: 5px; }"
    ".btn:hover { background-color: #45a049; }"
    ".output-data { background-color: #f9f9f9; padding: 10px; border-radius: "
    "5px; font-family: monospace; }"
    ".status { padding: 10px; border-radius: 5px; margin: 10px 0; }"
    ".status.connected { background-color: #d4edda; color: #155724; }"
    ".status.disconnected { background-color: #f8d7da; color: #721c24; }"
    "</style>"
    "</head><body>"
    "<div class=\"container\">"
    "<h1>ESP32 Robot Control Panel</h1>"

    "<div id=\"status\" class=\"status disconnected\">Disconnected</div>"

    "<div class=\"section\">"
    "<h3>Control Parameters</h3>"

    "<div class=\"input-group\">"
    "<label>d1:</label><input type=\"number\" id=\"d1\" step=\"0.1\" "
    "value=\"1\">"
    "<label>theta2:</label><input type=\"number\" id=\"theta2\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>theta3:</label><input type=\"number\" id=\"theta3\" step=\"0.1\" "
    "value=\"0\">"
    "<label>theta4:</label><input type=\"number\" id=\"theta4\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>theta5:</label><input type=\"number\" id=\"theta5\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>x:</label><input type=\"number\" id=\"x\" step=\"0.1\" value=\"0\">"
    "<label>y:</label><input type=\"number\" id=\"y\" step=\"0.1\" value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>z:</label><input type=\"number\" id=\"z\" step=\"0.1\" value=\"0\">"
    "<label>w:</label><input type=\"number\" id=\"w\" step=\"0.1\" value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>xip:</label><input type=\"number\" id=\"xip\" step=\"0.1\" "
    "value=\"0\">"
    "<label>yip:</label><input type=\"number\" id=\"yip\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>zip:</label><input type=\"number\" id=\"zip\" step=\"0.1\" "
    "value=\"0\">"
    "<label>wip:</label><input type=\"number\" id=\"wip\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>xfp:</label><input type=\"number\" id=\"xfp\" step=\"0.1\" "
    "value=\"0\">"
    "<label>yfp:</label><input type=\"number\" id=\"yfp\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>zfp:</label><input type=\"number\" id=\"zfp\" step=\"0.1\" "
    "value=\"0\">"
    "<label>wfp:</label><input type=\"number\" id=\"wfp\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"input-group\">"
    "<label>hp:</label><input type=\"number\" id=\"hp\" step=\"0.1\" "
    "value=\"0\">"
    "</div>"
    "<div class=\"checkbox-group\">"
    "<label><input type=\"checkbox\" id=\"dir_kinematics_on\"> Direct "
    "Kinematics</label>"
    "<label><input type=\"checkbox\" id=\"inv_kinematics_on\"> Inverse "
    "Kinematics</label>"
    "<label><input type=\"checkbox\" id=\"pick_and_place_on\"> Pick and "
    "Place</label>"
    "</div>"
    "<button class=\"btn\" onclick=\"sendData()\">Send Parameters</button>"
    "</div>"

    "<div class=\"section\">"
    "<h3>System Output (Real-time)</h3>"
    "<div id=\"output\" class=\"output-data\">Waiting for data...</div>"
    "</div>"

    "</div>"

    "<script>"
    "console.log(\"Script loaded!\");"
    "let connected = false;"

    "function updateStatus(isConnected) {"
    "  const statusDiv = document.getElementById('status');"
    "  if (isConnected) {"
    "    statusDiv.textContent = 'Connected';"
    "    statusDiv.className = 'status connected';"
    "  } else {"
    "    statusDiv.textContent = 'Disconnected';"
    "    statusDiv.className = 'status disconnected';"
    "  }"
    "  connected = isConnected;"
    "}"

    "function sendData() {"
    "  const data = {"
    "    d1: parseFloat(document.getElementById('d1').value),"
    "    theta2: parseFloat(document.getElementById('theta2').value),"
    "    theta3: parseFloat(document.getElementById('theta3').value),"
    "    theta4: parseFloat(document.getElementById('theta4').value),"
    "    theta5: parseFloat(document.getElementById('theta5').value),"
    "    x: parseFloat(document.getElementById('x').value),"
    "    y: parseFloat(document.getElementById('y').value),"
    "    z: parseFloat(document.getElementById('z').value),"
    "    w: parseFloat(document.getElementById('w').value),"
    "    xip: parseFloat(document.getElementById('xip').value),"
    "    yip: parseFloat(document.getElementById('yip').value),"
    "    zip: parseFloat(document.getElementById('zip').value),"
    "    wip: parseFloat(document.getElementById('wip').value),"
    "    xfp: parseFloat(document.getElementById('xfp').value),"
    "    yfp: parseFloat(document.getElementById('yfp').value),"
    "    zfp: parseFloat(document.getElementById('zfp').value),"
    "    wfp: parseFloat(document.getElementById('wfp').value),"
    "    hp: parseFloat(document.getElementById('hp').value),"
    "    dir_kinematics_on: "
    "document.getElementById('dir_kinematics_on').checked ? 1 : 0,"
    "    inv_kinematics_on: "
    "document.getElementById('inv_kinematics_on').checked ? 1 : 0,"
    "    pick_and_place_on: "
    "document.getElementById('pick_and_place_on').checked ? 1 : 0"
    "  };"
    ""
    "  fetch('/api/control', {"
    "    method: 'POST',"
    "    headers: {'Content-Type': 'application/json'},"
    "    body: JSON.stringify(data)"
    "  })"
    "  .then(response => response.json())"
    "  .then(result => {"
    "    console.log('Data sent successfully:', result);"
    "    updateStatus(true);"
    "  })"
    "  .catch(error => {"
    "    console.error('Error sending data:', error);"
    "    updateStatus(false);"
    "  });"
    "}"

    "function fetchOutputData() {"
    "  fetch('/api/output')"
    "  .then(response => response.json())"
    "  .then(data => {"
    "    const output = document.getElementById('output');"
    "    output.innerHTML = "
    "      'Horizontal Load: ' + data.horizontal_load + '<br>' +"
    "      'Vertical Load: ' + data.vertical_load + '<br>' +"
    "      'Encoder0: ' + data.encoder0 + '<br>' +"
    "      'Encoder1: ' + data.encoder1 + '<br>' +"
    "      'Encoder2: ' + data.encoder2 + '<br>' +"
    "      'Encoder3: ' + data.encoder3 + '<br>' +"
    "      'Switch0: ' + data.switch0 + '<br>' +"
    "      'Switch1: ' + data.switch1 + '<br>' +"
    "      'Position x: ' + data.x + '<br>' +"
    "      'Position y: ' + data.y + '<br>' +"
    "      'Position z: ' + data.z + '<br>' +"
    "      'Position w: ' + data.w + '<br>' +"
    "      'd1: ' + data.d1 + '<br>' +"
    "      'theta2: ' + data.theta2 + '<br>' +"
    "      'theta3: ' + data.theta3 + '<br>' +"
    "      'theta4: ' + data.theta4 + '<br>' +"
    "      'theta5: ' + data.theta5;"
    "    updateStatus(true);"
    "  })"
    "  .catch(error => {"
    "    console.error('Error fetching output data:', error);"
    "    updateStatus(false);"
    "  });"
    "}"

"console.log('Setting up fetch interval...');"
"setInterval(function() {"
"  console.log('Calling fetchOutputData...');"
"  fetchOutputData();"
"}, 2000);"

"console.log('Calling fetchOutputData manually...');"
"fetchOutputData();"

"</script>"
"</body></html>";

// HTTP GET handler for the main page
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// HTTP POST handler for control data
static esp_err_t control_post_handler(httpd_req_t *req) {
  char buf[512];
  int ret, remaining = req->content_len;

  if (remaining >= sizeof(buf)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
    return ESP_FAIL;
  }

  ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
  if (ret <= 0) {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Request timeout");
    }
    return ESP_FAIL;
  }

  buf[ret] = '\0';

  // Parse JSON data
  cJSON *json = cJSON_Parse(buf);
  if (json == NULL) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }

  // Create mutex if not exists
  if (g_user_input_mutex == NULL) {
    g_user_input_mutex = xSemaphoreCreateMutex();
  }

  // Update user input data with mutex protection
  if (xSemaphoreTake(g_user_input_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    cJSON *item;

    if ((item = cJSON_GetObjectItem(json, "d1")) != NULL) {
      client_input_data.d1 = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "theta2")) != NULL) {
      client_input_data.theta2 = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "theta3")) != NULL) {
      client_input_data.theta3 = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "theta4")) != NULL) {
      client_input_data.theta4 = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "theta5")) != NULL) {
      client_input_data.theta5 = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "x")) != NULL) {
      client_input_data.x = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "y")) != NULL) {
      client_input_data.y = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "z")) != NULL) {
      client_input_data.z = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "w")) != NULL) {
      client_input_data.w = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "xip")) != NULL) {
      client_input_data.xip = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "yip")) != NULL) {
      client_input_data.yip = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "zip")) != NULL) {
      client_input_data.zip = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "wip")) != NULL) {
      client_input_data.wip = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "xfp")) != NULL) {
      client_input_data.xfp = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "yfp")) != NULL) {
      client_input_data.yfp = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "zfp")) != NULL) {
      client_input_data.zfp = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "wfp")) != NULL) {
      client_input_data.wfp = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "hp")) != NULL) {
      client_input_data.hp = (float)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "dir_kinematics_on")) != NULL) {
      client_input_data.dir_kinematics_on = (int)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "inv_kinematics_on")) != NULL) {
      client_input_data.inv_kinematics_on = (int)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "pick_and_place_on")) != NULL) {
      client_input_data.pick_and_place_on = (int)cJSON_GetNumberValue(item);
    }

    xSemaphoreGive(g_user_input_mutex);

    ESP_LOGI(TAG,
             "Updated input data: d1=%.2f, theta2=%.2f, theta3=%.2f, "
             "theta4=%.2f, theta5=%.2f",
             client_input_data.d1, client_input_data.theta2,
             client_input_data.theta3, client_input_data.theta4,
             client_input_data.theta5);
    ESP_LOGI(TAG, "Position: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
             client_input_data.x, client_input_data.y, client_input_data.z,
             client_input_data.w);
    ESP_LOGI(
        TAG,
        "Xip=%.2f, Yip=%.2f, Zip=%.2f, Wip=%.2f, Xfp=%.2f, Yfp=%.2f, "
        "Zfp=%.2f, Wfp=%.2f, Hp=%.2f",
        client_input_data.xip, client_input_data.yip, client_input_data.zip,
        client_input_data.wip, client_input_data.xfp, client_input_data.yfp,
        client_input_data.zfp, client_input_data.wfp, client_input_data.hp);
    ESP_LOGI(TAG, "Kinematics: dir=%d, inv=%d, pap=%d",
             client_input_data.dir_kinematics_on,
             client_input_data.inv_kinematics_on,
             client_input_data.pick_and_place_on);
  }

  cJSON_Delete(json);

  // Send success response
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, "{\"status\":\"success\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// HTTP GET handler for output data
static esp_err_t output_get_handler(httpd_req_t *req) {
  system_output_data local_output;

  // Get current system output data with mutex protection
  if (xSemaphoreTake(g_system_output_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    memcpy(&local_output, &system_output_data_values,
           sizeof(system_output_data));
    ESP_LOGI(TAG, "Sending output!");
    xSemaphoreGive(g_system_output_mutex);
  } else {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Failed to get data");
    return ESP_FAIL;
  }

  // Create JSON response
  char json_buffer[1024];
  snprintf(json_buffer, sizeof(json_buffer),
           "{"
           "\"horizontal_load\":%.3f,"
           "\"vertical_load\":%.3f,"
           "\"encoder0\":%.3f,"
           "\"encoder1\":%.3f,"
           "\"encoder2\":%.3f,"
           "\"encoder3\":%.3f,"
           "\"switch0\":%d,"
           "\"switch1\":%d,"
           "\"x\":%.3f,"
           "\"y\":%.3f,"
           "\"z\":%.3f,"
           "\"w\":%.3f,"
           "\"d1\":%.3f,"
           "\"theta2\":%.3f,"
           "\"theta3\":%.3f,"
           "\"theta4\":%.3f,"
           "\"theta5\":%.3f"
           "}",
           local_output.horizontal_load, local_output.vertical_load,
           local_output.encoder0, local_output.encoder1, local_output.encoder2,
           local_output.encoder3, local_output.switch0, local_output.switch1,
           local_output.x, local_output.y, local_output.z, local_output.w,
           local_output.d1, local_output.theta2, local_output.theta3,
           local_output.theta4, local_output.theta5);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json_buffer, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// URI handlers
static const httpd_uri_t root = {.uri = "/",
                                 .method = HTTP_GET,
                                 .handler = root_get_handler,
                                 .user_ctx = NULL};

static const httpd_uri_t control_api = {.uri = "/api/control",
                                        .method = HTTP_POST,
                                        .handler = control_post_handler,
                                        .user_ctx = NULL};

static const httpd_uri_t output_api = {.uri = "/api/output",
                                       .method = HTTP_GET,
                                       .handler = output_get_handler,
                                       .user_ctx = NULL};

// Function to start the web server
static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &control_api);
    httpd_register_uri_handler(server, &output_api);
    ESP_LOGI(TAG, "Registering URI handlers");
    return server;
  }

  ESP_LOGI(TAG, "Error starting server!");
  return NULL;
}

void update_system_output_data(system_output_data *new_data) {
  ESP_LOGI("update_system_output_data", "updading data!");
  if (g_system_output_mutex &&
      xSemaphoreTake(g_system_output_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    memcpy(&system_output_data_values, new_data, sizeof(system_output_data));
    xSemaphoreGive(g_system_output_mutex);
  } else {
    ESP_LOGE(
        TAG,
        "Failed to take g_system_output_mutex in update_system_output_data");
  }
}

// Function to get current user input data (new function for your control loop)
void get_user_input_data(user_input_data *data) {
  if (g_user_input_mutex &&
      xSemaphoreTake(g_user_input_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    memcpy(data, &client_input_data, sizeof(user_input_data));
    xSemaphoreGive(g_user_input_mutex);
  } else {
    ESP_LOGE(TAG, "Failed to take g_user_input_mutex in get_user_input_data");
    // Optionally, initialize data to default values if mutex cannot be taken
    memset(data, 0, sizeof(user_input_data));
  }
}

void wifi_initialization_func() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  // Create mutexes
  g_system_output_mutex = xSemaphoreCreateMutex();
  g_user_input_mutex = xSemaphoreCreateMutex();

  if (g_system_output_mutex == NULL || g_user_input_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutexes");
    return;
  }

  // Initialize WiFi (assuming you have an init_wifi function)
  esp_err_t wifi_result = init_wifi(&esp_net_conf);
  if (wifi_result != ESP_OK) {
    ESP_LOGE(TAG, "Exiting: Wi-Fi connection failed.");
    esp_deep_sleep_start();
    return;
  }

  // Start the web server instead of TCP tasks
  httpd_handle_t server = start_webserver();
  if (server == NULL) {
    ESP_LOGE(TAG, "Failed to start web server");
    return;
  }

  ESP_LOGI(
      TAG,
      "Web server started successfully. Connect to: http://[ESP32_IP_ADDRESS]");
  return;
}
///////////////////////////
////// switch /////////////
///////////////////////////

#define GPIO_SWITCH_0 GPIO_NUM_15
#define GPIO_SWITCH_1 GPIO_NUM_32

gpio_config_t switch_0_io_conf = {
    .pin_bit_mask = (1ULL << GPIO_SWITCH_0), // Set the GPIO pin
    .mode = GPIO_MODE_INPUT,                 // Set as input mode
    .pull_up_en = GPIO_PULLUP_ENABLE,        // Enable pull-up resistor
    .pull_down_en = GPIO_PULLDOWN_DISABLE,   // Disable pull-down
    .intr_type = GPIO_INTR_DISABLE,
};

gpio_config_t switch_1_io_conf = {
    .pin_bit_mask = (1ULL << GPIO_SWITCH_1),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

switch_t switch_0 = {
    .config = &switch_0_io_conf,
    .gpio_pin = GPIO_SWITCH_0,
    .is_pressed = false,
};

switch_t switch_1 = {
    .config = &switch_1_io_conf,
    .gpio_pin = GPIO_SWITCH_1,
    .is_pressed = false,
};

void switch_initialization_task() {
  switch_init(&switch_0);
  switch_init(&switch_1);

  // TODO: one task for all switch
  xTaskCreate(switch_task, "switch_update_task", 4096, &switch_0, 5, NULL);
  xTaskCreate(switch_task, "switch_update_task", 4096, &switch_1, 5, NULL);
}
//////////////////////////////////
////// drivers/i2c_bus ///////////
//////////////////////////////////

static SemaphoreHandle_t i2c_mutex = NULL;

static i2c_master_bus_handle_t master_bus_handle;
static i2c_master_bus_config_t master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .sda_io_num = GPIO_I2C_SDA,
    .scl_io_num = GPIO_I2C_SCL,
    .glitch_ignore_cnt = I2C_MASTER_GLITCH_IGNORE_CNT,
    .flags.enable_internal_pullup = I2C_ENABLE_INTERNAL_PULLUP,
};

static i2c_master_config_t i2c_master_conf = {
    .bus_cfg = &master_bus_config,
    .bus_handle = &master_bus_handle,
    /* .i2c_mutex = &i2c_mutex, */
    .timeout_ticks = portMAX_DELAY,
};

static i2c_master_dev_handle_t encoder_0_handle;
static i2c_master_dev_handle_t encoder_1_handle;
static i2c_master_dev_handle_t encoder_2_handle;
static i2c_master_dev_handle_t encoder_3_handle;
static i2c_master_dev_handle_t tca_handle;

static i2c_device_config_t encoder_0_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_AS5600,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};

static i2c_device_config_t encoder_1_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_AS5600,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};
static i2c_device_config_t encoder_2_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_AS5600,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};
static i2c_device_config_t encoder_3_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_AS5600,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};

static i2c_device_config_t tca_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_ADDR_TCA,
    .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
};

static i2c_slave_bus_params i2c_slave_conf_encoder_0 = {
    .dev_cfg = &encoder_0_cfg,
    .dev_handle = &encoder_0_handle,
};

static i2c_slave_bus_params i2c_slave_conf_encoder_1 = {
    .dev_cfg = &encoder_1_cfg,
    .dev_handle = &encoder_1_handle,
};

static i2c_slave_bus_params i2c_slave_conf_encoder_2 = {
    .dev_cfg = &encoder_2_cfg,
    .dev_handle = &encoder_2_handle,
};

static i2c_slave_bus_params i2c_slave_conf_encoder_3 = {
    .dev_cfg = &encoder_3_cfg,
    .dev_handle = &encoder_3_handle,
};
static i2c_slave_bus_params i2c_slave_conf_tca = {
    .dev_cfg = &tca_cfg,
    .dev_handle = &tca_handle,
};

static encoder_t encoder_0 = {
    .label = "Encoder 0",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_0,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 0,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,

    .initial_offset = 3906,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 0,
    .is_calibrated = false,
    .switch_n = &switch_0,
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

static encoder_t encoder_1 = {
    .label = "Encoder 1",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_1,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 1,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,

    .initial_offset = -5876,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 62.0 / 18.0,
    .test_offset = 706,
    .is_calibrated = false,
    .switch_n = &switch_1,
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

static encoder_t encoder_2 = {
    .label = "Encoder 2",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_2,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 2,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,

    .initial_offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 1,
    .test_offset = 706,
    .is_calibrated = false,
    .switch_n = &switch_1,
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

static encoder_t encoder_3 = {
    .label = "Encoder 3",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_3,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 3,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,

    .initial_offset = 0,
    .reverse = false,
    .current_reading = 0,
    .accumulated_steps = 0,
    .is_inverted = 1,
    .gear_ratio = 1,
    .test_offset = 706,
    .is_calibrated = false,
    /* .switch_n = &switch_1, */
    .angle_degrees = 0,

    .encoder_resolution = 4096, // Will default to 4096 if 0
    .offset_degrees = 0.0f,
    .motor_angle_degrees = 0.0f,
};

#include "stdlib.h"

void encoder_initialization_task() {

  i2c_mutex = xSemaphoreCreateMutex();
  if (i2c_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create I2C mutex!");
    return;
  }

  i2c_master_conf.i2c_mutex = i2c_mutex;

  ESP_ERROR_CHECK(
      i2c_new_master_bus(i2c_master_conf.bus_cfg, i2c_master_conf.bus_handle));
  ESP_LOGI(TAG, "I2C bus handle: %p", *i2c_master_conf.bus_handle);

  ESP_ERROR_CHECK(i2c_master_bus_add_device(*i2c_master_conf.bus_handle,
                                            i2c_slave_conf_tca.dev_cfg,
                                            i2c_slave_conf_tca.dev_handle));

  ESP_LOGI(TAG, "TCA handle pointer: %p", (void *)encoder_0.i2c_tca);
  ESP_LOGI(TAG, "TCA device handle: %p", (void *)encoder_0.i2c_tca->dev_handle);

  if (encoder_0.i2c_tca != NULL && encoder_0.i2c_tca->dev_handle != NULL) {
    ESP_ERROR_CHECK(tca_select_channel(encoder_0.tca_channel,
                                       encoder_0.i2c_tca->dev_handle));
  } else {
    ESP_LOGE(TAG, "TCA device handle not initialized yet.");
  }

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_0.tca_channel, encoder_0.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_0.dev_cfg,
      i2c_slave_conf_encoder_0.dev_handle));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_1.tca_channel, encoder_1.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_1.dev_cfg,
      i2c_slave_conf_encoder_1.dev_handle));
  ESP_ERROR_CHECK(encoder_init(&encoder_1));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_2.tca_channel, encoder_2.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_2.dev_cfg,
      i2c_slave_conf_encoder_2.dev_handle));
  ESP_ERROR_CHECK(encoder_init(&encoder_2));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_3.tca_channel, encoder_3.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_3.dev_cfg,
      i2c_slave_conf_encoder_3.dev_handle));
  ESP_ERROR_CHECK(encoder_init(&encoder_3));

  xTaskCreate(encoder_task, "encoder0_task", 4096, &encoder_0, 5, NULL);
  xTaskCreate(encoder_task, "encoder1_task", 4096, &encoder_1, 5, NULL);
  xTaskCreate(encoder_task, "encoder2_task", 4096, &encoder_2, 5, NULL);
  xTaskCreate(encoder_task, "encoder3_task", 4096, &encoder_3, 5, NULL);

  /* xTaskCreate(encoder_try_calibration_task, "encoder0_cal_task", 4096, */
  /*             &encoder_0, 5, NULL); */

  ESP_LOGE(TAG, "exiting encoder_initialization_task");
}

//////////////////////////////////
////// network/wifi_manager //////
//////////////////////////////////

/* Motor	mcpwm_unit	mcpwm_timer	mcpwm_opr */
/* 1	0	0	0 */
/* 2	0	1	1 */
/* 3	0	2	2 */
/* 4	1	0	0 */
/* 5	1	1	1 */
/* 6	1	2	2 */

motor_mcpwm_vars mcpwm_vars_x = {
    .group_unit = 0,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_y = {
    .group_unit = 0,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_z = {
    .group_unit = 1,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_a = {
    .group_unit = 1,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};

motor_mcpwm_vars mcpwm_vars_b = {
    .group_unit = 1,
    .timer = NULL,
    .operator= NULL,
    .comparator = NULL,
    .generator = NULL,
    .esp_timer_handle = 0,
    .mcpwm_min_period_ticks = MCPWM_MIN_PERIOD_TICKS,
    .mcpwm_max_period_ticks = MCPWM_MAX_PERIOD_TICKS,
    .pwm_resolution_hz = PWM_RESOLUTION_HZ,
};
motor_pwm_vars_t pwm_vars_x = {
    .step_count = 0,
    .max_freq = 1200,
    .min_freq = 0,
    .max_accel = 3000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

motor_pwm_vars_t pwm_vars_y = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 3000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = true,
};

motor_pwm_vars_t pwm_vars_z = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 3000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = true,
};

motor_pwm_vars_t pwm_vars_a = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 1000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = true,
};

motor_pwm_vars_t pwm_vars_b = {
    .step_count = 0,
    .max_freq = 1000,
    .min_freq = 0,
    .max_accel = 5000,
    .current_freq_hz = 0,
    .target_freq_hz = 0,
    .dir_is_reversed = false,
};

pid_controller_t pid_motor_x = {
    .Kp = 0.2,    // 1
    .Ki = 0.0025, // 0.01
    .Kd = 0.0,    // 0.2
};

pid_controller_t pid_motor_y = {
    .Kp = 0.2 * 4, // 1
    .Ki = 0.0025,  // 0.01
    .Kd = 0.0,     // 0.2
};

pid_controller_t pid_motor_z = {
    .Kp = 0.1 * 5, // 1
    .Ki = 0.0,     // 0.01
    .Kd = 0.0,     // 0.2
};

pid_controller_t pid_motor_a = {
    .Kp = 0.5, // 1
    .Ki = 0.0, // 0.01
    .Kd = 0.0, // 0.2
};

pid_controller_t pid_motor_b = {
    .Kp = 0.5, // 1
    .Ki = 0.0, // 0.01
    .Kd = 0.0, // 0.2
};

motor_control_vars control_vars_x = {
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_y,
    .ref_switch = &switch_0,
};

motor_control_vars control_vars_y = {
    .ref_encoder = &encoder_0,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_y,
    .ref_switch = &switch_1,
};

motor_control_vars control_vars_z = {
    .ref_encoder = &encoder_1,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_z,
    .ref_switch = &switch_1,
};

motor_control_vars control_vars_a = {
    .ref_encoder = &encoder_2,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_a,
    /* .ref_switch = &switch_1, */
};

motor_control_vars control_vars_b = {
    .ref_encoder = &encoder_3,
    .encoder_target_pos = 0,
    .enable_pid = false,
    .pid = &pid_motor_b,
    /* .ref_switch = &switch_1, */
};

motor_t motor_x = {
    .label = MOTOR_X_LABEL,
    .id = MOTOR_X_ID,
    .gpio_stp = GPIO_X_STP,
    .gpio_dir = GPIO_X_DIR,
    .mcpwm_vars = &mcpwm_vars_x,
    .pwm_vars = &pwm_vars_x,
    .control_vars = &control_vars_x,
    .is_inverted = true,
};

motor_t motor_y = {
    .label = MOTOR_Y_LABEL,
    .id = MOTOR_Y_ID,
    .gpio_stp = GPIO_Y_STP,
    .gpio_dir = GPIO_Y_DIR,
    .mcpwm_vars = &mcpwm_vars_y,
    .pwm_vars = &pwm_vars_y,
    .control_vars = &control_vars_y,
    .is_inverted = true,
};

motor_t motor_z = {
    .label = MOTOR_Z_LABEL,
    .id = MOTOR_Z_ID,
    .gpio_stp = GPIO_Z_STP,
    .gpio_dir = GPIO_Z_DIR,
    .mcpwm_vars = &mcpwm_vars_z,
    .pwm_vars = &pwm_vars_z,
    .control_vars = &control_vars_z,
    .is_inverted = true,
};

motor_t motor_a = {
    .label = MOTOR_A_LABEL,
    .id = MOTOR_A_ID,
    .gpio_stp = GPIO_A_STP,
    .gpio_dir = GPIO_A_DIR,
    .mcpwm_vars = &mcpwm_vars_a,
    .pwm_vars = &pwm_vars_a,
    .control_vars = &control_vars_a,
    .is_inverted = false,
};

motor_t motor_b = {
    .label = MOTOR_B_LABEL,
    .id = MOTOR_B_ID,
    .gpio_stp = GPIO_B_STP,
    .gpio_dir = GPIO_B_DIR,
    .mcpwm_vars = &mcpwm_vars_b,
    .pwm_vars = &pwm_vars_b,
    .control_vars = &control_vars_b,
    .is_inverted = false,
};

void motor_initialization_task() {
  esp_err_t err;

  motor_init_dir(&motor_x);
  motor_create_pwm(&motor_x);
  err = motor_init_timer(&motor_x);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor X");
  }
  motor_x.pwm_vars->target_steps = 0;
  motor_x.pwm_vars->step_counting_enabled = true;

  motor_init_dir(&motor_y);
  motor_create_pwm(&motor_y);
  err = motor_init_timer(&motor_y);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor Y");
  }

  motor_init_dir(&motor_z);
  motor_create_pwm(&motor_z);
  err = motor_init_timer(&motor_z);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor Z");
  }

  motor_init_dir(&motor_a);
  motor_create_pwm(&motor_a);
  err = motor_init_timer(&motor_a);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor A");
  }
  motor_init_dir(&motor_b);
  motor_create_pwm(&motor_b);
  err = motor_init_timer(&motor_b);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize timer for motor B");
  }

  ESP_LOGI(TAG, "All motors initialized successfully");
  /* xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_x, 5, NULL); */

  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_y, 5, NULL);
  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_z, 5, NULL);
  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_a, 5, NULL);
  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &motor_b, 5, NULL);
  return;
}

void calibration_initialization_task() {
  gpio_set_level(motor_x.gpio_dir, true);
  gpio_set_level(motor_y.gpio_dir, true);
  gpio_set_level(motor_z.gpio_dir, false);
  gpio_set_level(motor_a.gpio_dir, false);
  gpio_set_level(motor_b.gpio_dir, false);

  encoder_zero_position(motor_a.control_vars->ref_encoder);
  encoder_zero_position(motor_b.control_vars->ref_encoder);

  motor_a.control_vars->encoder_target_pos = 0;
  motor_b.control_vars->encoder_target_pos = 0;

  motor_set_target_frequency(&motor_x, motor_x.pwm_vars->max_freq);
  while (!motor_x.control_vars->ref_switch->is_pressed) {
    ESP_LOGI("calibration_initialization_task", "calibrating motor_x");
    vTaskDelay(20);
  }
  motor_set_target_frequency(&motor_x, 0);
  /* motor_move_steps(&motor_x, 6400 * 10 / 6 * 5, motor_x.pwm_vars->max_freq);
   */
  motor_set_current_position(&motor_x, 0); // Start at position 0
  motor_move_to_position(&motor_x, 6600, motor_x.pwm_vars->max_freq);

  motor_set_target_frequency(&motor_y, motor_y.pwm_vars->max_freq / 4);
  motor_y.pwm_vars->target_freq_hz = motor_y.pwm_vars->max_freq / 4;
  while (!motor_y.control_vars->ref_switch->is_pressed) {
    ESP_LOGI("calibration_initialization_task", "calibrating motor_y");
    vTaskDelay(20);
  }
  motor_set_target_frequency(&motor_y, 0);
  encoder_zero_position(motor_y.control_vars->ref_encoder);
  motor_y.control_vars->encoder_target_pos = 0;

  vTaskDelay(2000 / portTICK_PERIOD_MS);

  while (!motor_z.control_vars->ref_switch->is_pressed) {
    motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq / 4);
    ESP_LOGI("calibration_initialization_task", "calibrating motor_z");
    vTaskDelay(20);
  }

  ESP_LOGI("calibration_initialization_task", "finished motor_z cal");
  motor_set_target_frequency(&motor_z, 0);
  encoder_zero_position(motor_z.control_vars->ref_encoder);
  motor_z.control_vars->encoder_target_pos = 0;
  motor_y.control_vars->encoder_target_pos = 0;
  ESP_LOGW("calibration", "\n\n\nAll calibrations done!\n\n\n");

  return;
}

//////////////////////////////////
////// Call functions ////////////
//////////////////////////////////

void loop_scara_task() {
  while (1) {
    ESP_LOGI("loop_scara_task", "dir 1");
    /* gpio_set_level(motor_x.gpio_dir, 1); */
    /* gpio_set_level(motor_x.gpio_stp, 1); */
    /**/
    /* gpio_set_level(motor_y.gpio_dir, 1); */
    /* gpio_set_level(motor_y.gpio_stp, 1); */
    /**/
    /* gpio_set_level(motor_z.gpio_dir, 1); */
    /* gpio_set_level(motor_z.gpio_stp, 1); */

    /* gpio_set_level(motor_a.gpio_dir, 1); */
    /* gpio_set_level(motor_a.gpio_stp, 1); */

    /* gpio_set_level(motor_b.gpio_stp, 1); */
    /* gpio_set_level(motor_b.gpio_dir, 1); */

    /* gpio_set_level(motor_z.gpio_dir, 1); */
    /* gpio_set_level(motor_a.gpio_dir, 1); */
    /* gpio_set_level(motor_b.gpio_dir, 1); */

    gpio_set_level(motor_x.gpio_dir, 1);
    gpio_set_level(motor_y.gpio_dir, 1);
    gpio_set_level(motor_z.gpio_dir, 1);
    gpio_set_level(motor_a.gpio_dir, 1);
    gpio_set_level(motor_b.gpio_dir, 1);

    motor_set_target_frequency(&motor_x, motor_x.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_y, motor_y.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_a, motor_a.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_b, motor_b.pwm_vars->max_freq);
    /* motor_move_steps(&motor_x, -4000, motor_x.pwm_vars->max_freq); */
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    motor_set_target_frequency(&motor_x, 0);
    motor_set_target_frequency(&motor_y, 0);
    motor_set_target_frequency(&motor_z, 0);
    motor_set_target_frequency(&motor_a, 0);
    motor_set_target_frequency(&motor_b, 0);
    /* motor_move_steps(&motor_x, 4000, motor_x.pwm_vars->max_freq); */
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI("loop_scara_task", "dir 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    gpio_set_level(motor_y.gpio_dir, 0);
    gpio_set_level(motor_z.gpio_dir, 0);
    gpio_set_level(motor_a.gpio_dir, 0);
    gpio_set_level(motor_b.gpio_dir, 0);
    /* motor_move_steps(&motor_x, -1000, motor_x.pwm_vars->max_freq); */

    motor_set_target_frequency(&motor_x, motor_x.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_y, motor_y.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_z, motor_z.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_a, motor_a.pwm_vars->max_freq);
    motor_set_target_frequency(&motor_b, motor_b.pwm_vars->max_freq);
    /* motor_move_steps(&motor_x, 1000, motor_x.pwm_vars->max_freq); */
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    motor_set_target_frequency(&motor_x, 0);
    motor_set_target_frequency(&motor_y, 0);
    motor_set_target_frequency(&motor_z, 0);
    motor_set_target_frequency(&motor_a, 0);
    motor_set_target_frequency(&motor_b, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

#include "loadcell.h"

// GPIO pin definitions for HX711 #0
#define HX711_0_DATA_PIN GPIO_NUM_4
#define HX711_0_CLOCK_PIN GPIO_NUM_2

// GPIO pin definitions for HX711 #1
#define HX711_1_DATA_PIN GPIO_NUM_13
#define HX711_1_CLOCK_PIN GPIO_NUM_12

hx711_t hx711_0 = {
    .label = "sensor_0",
    .data_pin = HX711_0_DATA_PIN,
    .clock_pin = HX711_0_CLOCK_PIN,
    .gain = HX711_GAIN_64,
    .tare_offset = 0,
    .scale = 1.0f,
    .raw_read = 0,
};

hx711_t hx711_1 = {
    .label = "sensor_1",
    .data_pin = HX711_1_DATA_PIN,
    .clock_pin = HX711_1_CLOCK_PIN,
    .gain = HX711_GAIN_128,
    .tare_offset = 0,
    .scale = 1.0f,
    .raw_read = 0,
};

void hx711_initialization_func() {
  hx711_gpio_init(&hx711_0);
  hx711_gpio_init(&hx711_1);

  xTaskCreate(hx711_task, "hx711_0_task", 4096, &hx711_0, 5, NULL);
  xTaskCreate(hx711_task, "hx712_1_task", 4096, &hx711_1, 5, NULL);
  /* xTaskCreate(hx711_avg_task, "hx712_1_avg_task", 4096, &hx711_1, 5, NULL);
   */
  return;
}

void loop_scara_readings() {
  while (1) {
    ESP_LOGI("switch_0", "is pressed: %d", switch_0.is_pressed);
    ESP_LOGI("switch_1", "is pressed: %d", switch_1.is_pressed);

    ESP_LOGI(encoder_0.label, "value deg: %f", encoder_0.angle_degrees);
    ESP_LOGI(encoder_1.label, "value deg: %f", encoder_1.angle_degrees);
    ESP_LOGI(encoder_2.label, "value deg: %f", encoder_2.angle_degrees);
    ESP_LOGI(encoder_3.label, "value deg: %f", encoder_3.angle_degrees);

    ESP_LOGI(encoder_0.label, "steps: %d", encoder_0.accumulated_steps);
    ESP_LOGI(encoder_1.label, "steps: %d", encoder_1.accumulated_steps);
    ESP_LOGI(encoder_2.label, "steps: %d", encoder_2.accumulated_steps);
    ESP_LOGI(encoder_3.label, "steps: %d", encoder_3.accumulated_steps);

    ESP_LOGI("", "");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return;
}

typedef struct {
  float d1;
  float theta2_1;
  float theta2_2;
  float theta3_1;
  float theta3_2;
} inv_kin_results;

inv_kin_results inv_kin_results_t;
user_input_data current_input;

void calculate_inverse_kinematics_2(float x, float y, float z,
                                    inv_kin_results *results);
void update_target_positions() {
  while (1) {
    // direct kinematics is active
    if (client_input_data.dir_kinematics_on &&
        !client_input_data.inv_kinematics_on) {

      motor_move_to_position(&motor_x, 6600 * client_input_data.d1,
                             motor_x.pwm_vars->max_freq);

      motor_y.control_vars->encoder_target_pos =
          client_input_data.theta2 * -1 *
          motor_y.control_vars->ref_encoder->encoder_resolution / 360 *
          motor_y.control_vars->ref_encoder->gear_ratio;

      motor_z.control_vars->encoder_target_pos =
          client_input_data.theta3 *
              motor_z.control_vars->ref_encoder->encoder_resolution / 360 *
              motor_z.control_vars->ref_encoder->gear_ratio -
          motor_y.control_vars->encoder_target_pos;

      motor_a.control_vars->encoder_target_pos =
          client_input_data.theta4 *
          motor_a.control_vars->ref_encoder->encoder_resolution / 360 *
          motor_a.control_vars->ref_encoder->gear_ratio;

      motor_b.control_vars->encoder_target_pos =
          client_input_data.theta5 *
          motor_b.control_vars->ref_encoder->encoder_resolution / 360 *
          motor_b.control_vars->ref_encoder->gear_ratio;
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // inverse kinematics is on
    } else if (client_input_data.inv_kinematics_on &&
               !client_input_data.dir_kinematics_on) {
      calculate_inverse_kinematics_2(client_input_data.x, client_input_data.y,
                                     client_input_data.z, &inv_kin_results_t);
      /* ESP_LOGI("client_input_data", "x: %.2f", client_input_data.x); */
      /* ESP_LOGI("client_input_data", "y: %.2f", client_input_data.y); */
      /* ESP_LOGI("client_input_data", "z: %.2f", client_input_data.z); */
      /* ESP_LOGI("client_input_data", "w: %.2f", client_input_data.w); */

      motor_move_to_position(&motor_x, 6600 * inv_kin_results_t.d1,
                             motor_x.pwm_vars->max_freq);

      motor_y.control_vars->encoder_target_pos =
          inv_kin_results_t.theta2_1 * 180 / M_PI * -1 *
          motor_y.control_vars->ref_encoder->encoder_resolution / 360 *
          motor_y.control_vars->ref_encoder->gear_ratio;

      motor_z.control_vars->encoder_target_pos =
          inv_kin_results_t.theta3_1 *
              motor_z.control_vars->ref_encoder->encoder_resolution / 360 *
              motor_z.control_vars->ref_encoder->gear_ratio -
          motor_y.control_vars->encoder_target_pos;

      motor_a.control_vars->encoder_target_pos =
          client_input_data.theta4 *
          motor_a.control_vars->ref_encoder->encoder_resolution / 360 *
          motor_a.control_vars->ref_encoder->gear_ratio;

      motor_b.control_vars->encoder_target_pos =
          client_input_data.theta5 *
          motor_b.control_vars->ref_encoder->encoder_resolution / 360 *
          motor_b.control_vars->ref_encoder->gear_ratio;
      vTaskDelay(1000 / portTICK_PERIOD_MS);

    } else {

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

// dimenstions in meters
#define A1_DIM 4.9 / 100
#define A2_DIM 10.0 / 100
#define A3_DIM 10.25 / 100
#define D3_VAL 5.9 / 100
#define D4_VAL 15.0 / 100

#define D1_MIN_VAL 0
#define D1_MAX_VAL 45.0 / 100

// angles in degrees
#define THETA2_MIN_VAL -100
#define THETA2_MAX_VAL 100
#define THETA3_MIN_VAL -135
#define THETA3_MAX_VAL 135
#define THETA4_MIN_VAL -180
#define THETA4_MAX_VAL 180

/* #define THETA3_4_SUM_MAX_VAL */

void calculate_direct_kinematics(float d1, float theta2, float theta3,
                                 float theta4, system_output_data *data) {
  data->x = A1_DIM + A2_DIM * cos(theta2 * M_PI / 180) +
            A3_DIM * cos((theta2 + theta3) * M_PI / 180);
  data->y = A2_DIM * sin(theta2 * M_PI / 180) +
            A3_DIM * sin((theta2 + theta3) * M_PI / 180);
  data->z = d1 - D3_VAL + D4_VAL;
}

// Helper function to check if a point is reachable
bool is_point_reachable(float x, float y, float z) {
  float r2 = sqrt(pow(x - A1_DIM, 2) + pow(y, 2));
  float max_reach = A2_DIM + A3_DIM;
  float min_reach = fabs(A2_DIM - A3_DIM);

  bool result = r2 <= max_reach && r2 >= min_reach;
  if (!result)
    ESP_LOGW("is_point_reachable", "point (%.2f,%.2f,%.2f) is not reachable.",
             x, y, z);
  return (result);
}

void calculate_inverse_kinematics_2(float x, float y, float z,
                                    inv_kin_results *results) {

  // Calculate r2 - distance from second joint to target in XY plane
  float r2 = sqrt(pow(x - A1_DIM, 2) + pow(y, 2));

  // Check if target is reachable
  float max_reach = A2_DIM + A3_DIM;
  float min_reach = fabs(A2_DIM - A3_DIM);

  if (r2 > max_reach || r2 < min_reach) {
    ESP_LOGE("calculate_inverse_kinematics",
             "Target unreachable! r2=%.2f, min=%.2f, max=%.2f", r2, min_reach,
             max_reach);
    // TODO: return to default position
    /* results->d1 = NAN; */
    /* results->theta2_1 = NAN; */
    /* results->theta2_2 = NAN; */
    /* results->theta3_1 = NAN; */
    /* results->theta3_2 = NAN; */

    return;
  }

  // Calculate the argument for acos - this is the cosine of the angle between
  // links
  float cos_beta2 =
      (pow(r2, 2) - pow(A2_DIM, 2) - pow(A3_DIM, 2)) / (-2 * A2_DIM * A3_DIM);

  // Clamp the value to [-1, 1] to handle numerical precision issues
  cos_beta2 = fmaxf(-1.0f, fminf(1.0f, cos_beta2));

  float beta2 = acos(cos_beta2);

  // Calculate theta3 (elbow angle)
  results->theta3_1 = M_PI - beta2;    // Elbow up configuration
  results->theta3_2 = -(M_PI - beta2); // Elbow down configuration

  /* ESP_LOGI("calculate_inverse_kinematics", "starting debug prints!"); */
  /* ESP_LOGI("calculate_inverse_kinematics", "x: %.2f", x); */
  /* ESP_LOGI("calculate_inverse_kinematics", "y: %.2f", y); */
  /* ESP_LOGI("calculate_inverse_kinematics", "z: %.2f", z); */
  /* ESP_LOGI("calculate_inverse_kinematics", "r2: %.2f", r2); */
  /* ESP_LOGI("calculate_inverse_kinematics", "cos_beta2: %.2f", cos_beta2); */
  /* ESP_LOGI("calculate_inverse_kinematics", "beta2: %.2f", beta2); */
  /* ESP_LOGI("theta3_1", "%.2f", results->theta3_1); */
  /* ESP_LOGI("theta3_2", "%.2f", results->theta3_2); */

  // Calculate beta3 (angle to target from first joint)
  float beta3 = atan2(y, x - A1_DIM);

  // Calculate phi (angle correction for link 2)
  float cos_phi =
      (pow(A2_DIM, 2) + pow(r2, 2) - pow(A3_DIM, 2)) / (2 * A2_DIM * r2);
  cos_phi = fmaxf(-1.0f, fminf(1.0f, cos_phi)); // Clamp again
  float phi = acos(cos_phi);

  // Calculate theta2 (shoulder angle)
  results->theta2_1 = beta3 + phi; // For elbow up
  results->theta2_2 = beta3 - phi; // For elbow down

  // Set d1 (prismatic joint for Z-axis)
  results->d1 = z;

  /* ESP_LOGI("calculate_inverse_kinematics", "beta3: %.2f", beta3); */
  /* ESP_LOGI("calculate_inverse_kinematics", "phi: %.2f", phi); */
  /* ESP_LOGI("calculate_inverse_kinematics", "theta2_1: %.2f",
   * results->theta2_1); */
  /* ESP_LOGI("calculate_inverse_kinematics", "theta2_2: %.2f",
   * results->theta2_2); */
  /* ESP_LOGI("calculate_inverse_kinematics", ""); */
  /* ESP_LOGI("calculate_inverse_kinematics", ""); */
  /* ESP_LOGI("calculate_inverse_kinematics", ""); */
  return;
}

void update_monitoring_data(system_output_data *data) {
  data->horizontal_load = hx711_0.raw_read;
  data->vertical_load = hx711_1.raw_read;

  data->encoder0 = encoder_0.accumulated_steps;
  data->encoder1 = encoder_1.accumulated_steps;
  data->encoder2 = encoder_2.accumulated_steps;
  data->encoder3 = encoder_3.accumulated_steps;

  data->switch0 = switch_0.is_pressed;
  data->switch1 = switch_1.is_pressed;

  // variable parameters
  data->d1 = 0;

  // BUG: broken
  data->theta2 = -1 * encoder_get_angle_degrees(&encoder_0);
  data->theta3 = encoder_get_angle_degrees(&encoder_1) - data->theta2;
  data->theta4 = encoder_get_angle_degrees(&encoder_2);
  data->theta5 = encoder_get_angle_degrees(&encoder_3);

  // update x, y, z with direct kinematics
  calculate_direct_kinematics(data->d1, data->theta2, data->theta3,
                              data->theta4, data);

  data->w = data->theta5; // TODO: maybe change this
  update_system_output_data(data);
}

void print_system_output_data(system_output_data *data) {
  ESP_LOGI(TAG, "horizontal_load: %.2f", data->horizontal_load);
  ESP_LOGI(TAG, "vertical_load: %.2f", data->vertical_load);
  ESP_LOGI(TAG, "encoder0: %.2f", data->encoder0);
  ESP_LOGI(TAG, "encoder1: %.2f", data->encoder1);
  ESP_LOGI(TAG, "encoder2: %.2f", data->encoder2);
  ESP_LOGI(TAG, "encoder3: %.2f", data->encoder3);
  ESP_LOGI(TAG, "switch0: %d", data->switch0);
  ESP_LOGI(TAG, "switch1: %d", data->switch1);
  ESP_LOGI(TAG, "x: %.2f", data->x);
  ESP_LOGI(TAG, "y: %.2f", data->y);
  ESP_LOGI(TAG, "z: %.2f", data->z);
  ESP_LOGI(TAG, "w: %.2f", data->w);
  ESP_LOGI(TAG, "d1: %.2f", data->d1);
  ESP_LOGI(TAG, "theta2: %.2f", data->theta2);
  ESP_LOGI(TAG, "theta3: %.2f", data->theta3);
  ESP_LOGI(TAG, "theta4: %.2f", data->theta4);
  ESP_LOGI(TAG, "theta5: %.2f", data->theta5);
  ESP_LOGI("", "");
}

// TODO: make this the only loop for readings
void loop_scara_readings_2() {
  while (1) {
    update_monitoring_data(&system_new_output_data_values);
    get_user_input_data(&client_input_data); // Get web interface input

    /* print_system_output_data(&system_output_data_values); */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void init_scara() {
  // components
  wifi_initialization_func();
  switch_initialization_task();
  encoder_initialization_task();
  motor_initialization_task();
  // BUG: broken :C
  /* hx711_initialization_func(); */

  // test functions
  /* xTaskCreate(loop_scara_task, "testloop", 4096, NULL, 5, NULL); */
  /* xTaskCreate(loop_scara_readings, "testreadings", 4096, NULL, 5, NULL); */

  xTaskCreate(loop_scara_readings_2, "testreadings", 4096, NULL, 5, NULL);
  /* calibration_initialization_task(); */

  /* calculate_inverse_kinematics_2(0.10, 0.10, 0.10, &inv_kin_results_t); */
  /* vTaskDelay(pdMS_TO_TICKS(5000)); */
  xTaskCreate(update_target_positions, "update target positions", 4096, NULL, 5,
              NULL);
  ESP_LOGI(TAG, "init_scara completed");
}
