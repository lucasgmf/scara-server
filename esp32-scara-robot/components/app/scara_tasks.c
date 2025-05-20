#include "scara_tasks.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char *TAG = "scara_tasks";

#define TASK_UPDATE_MOTOR_PWM_PERIOD_MS 20

void task_update_motor_pwm(void *arg) {
  motor_t *motor_n = (motor_t *)arg;
  if (motor_n == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  TickType_t last_wake_time = xTaskGetTickCount();
  const float dt_sec = TASK_UPDATE_MOTOR_PWM_PERIOD_MS / 1000.0f;

  while (true) {
    /* ESP_LOGI(TAG, "Current freq: %.2f Hz, Target: %d Hz", */
    /*          motor_n->current_freq_hz, motor_n->target_freq_hz); */

    float diff = motor_n->target_freq_hz - motor_n->current_freq_hz;
    float step = motor_n->speed_hz * dt_sec;

    // Ramp frequency toward target
    if (fabsf(diff) < step) {
      motor_n->current_freq_hz = motor_n->target_freq_hz;
    } else {
      motor_n->current_freq_hz += (diff > 0) ? step : -step;
    }

    // Update step count estimate
    motor_n->step_count += (int)(motor_n->current_freq_hz * dt_sec);

    // Update PWM
    motor_delete_pwm(motor_n);
    if (motor_n->current_freq_hz > 0.0f) {
      motor_create_pwm(motor_n);
    }

    // Wait for next tick
    vTaskDelayUntil(&last_wake_time,
                    pdMS_TO_TICKS(TASK_UPDATE_MOTOR_PWM_PERIOD_MS));
  }
}

void encoder_task(void *arg) {
  encoder_t *enc = (encoder_t *)arg;
  while (1) {
    encoder_read_angle(enc);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  return;
}

#define MOTOR_CONTROL_TASK_PERIOD_MS 10

void motor_control_task(void *arg) {
  motor_control_loop_t *loop = (motor_control_loop_t *)arg;
  const TickType_t dt_ticks = pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS);
  const float dt_sec = 0.02f;

  while (1) {
    uint16_t raw = encoder_read_angle(loop->encoder);
    loop->current_position = (float)raw;

    float error = loop->target_position - loop->current_position;

    ESP_LOGI("motor_debug", "Target: %.2f, Current: %.2f, Error: %.2f",
             loop->target_position, loop->current_position, error);
    // Deadband: if error is small, stop the motor
    const float DEAD_BAND_THRESHOLD =
        2.0f; // Adjust as needed based on your encoder units
    if (fabsf(error) < DEAD_BAND_THRESHOLD) {
      loop->pid->integral = 0; // Optional: prevent integral windup
      loop->output_freq_hz = 0;
      motor_set_frequency(loop->motor, 0);
      vTaskDelay(dt_ticks);
      continue;
    }
    loop->pid->integral += error * dt_sec;
    float derivative = (error - loop->pid->prev_error) / dt_sec;
    loop->pid->prev_error = error;

    float output = loop->pid->Kp * error + loop->pid->Ki * loop->pid->integral +
                   loop->pid->Kd * derivative;

    // Clamp output to allowed frequency
    if (output > loop->max_output_freq_hz)
      output = loop->max_output_freq_hz;
    if (output < -loop->max_output_freq_hz)
      output = -loop->max_output_freq_hz;

    loop->output_freq_hz = fabsf(output);

    // Determine direction
    bool reverse = output < 0;
    gpio_set_level(loop->motor->gpio_dir,
                   loop->direction_reversed ? !reverse : reverse);

    // Apply new frequency to motor
    motor_set_frequency(loop->motor, (int)loop->output_freq_hz);

    vTaskDelay(dt_ticks);
  }
}

void tcp_server_task(void *arg) {
  network_configuration *net_conf = (network_configuration *)arg;
  if (net_conf == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  char rx_buffer[net_conf->rx_buffer_size];
  char addr_str[net_conf->addr_str_size];

  struct sockaddr_in dest_addr = {
      .sin_addr.s_addr = htonl(INADDR_ANY),
      .sin_family = AF_INET,
      .sin_port = htons(net_conf->port),
  };

  int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(TAG, "Socket created");

  int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    close(listen_sock);
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(TAG, "Socket bound, port %d", net_conf->port);

  err = listen(listen_sock, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
    close(listen_sock);
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    ESP_LOGI(TAG, "Waiting for connection...");
    struct sockaddr_in6 source_addr;
    socklen_t addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
      break;
    }

    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str,
                sizeof(addr_str) - 1);
    ESP_LOGI(TAG, "Connection from %s", addr_str);

    while (1) {
      int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
      if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        break;
      } else if (len == 0) {
        ESP_LOGI(TAG, "Connection closed");
        break;
      } else {
        rx_buffer[len] = 0;
        ESP_LOGI(TAG, "Received raw: %s", rx_buffer);

        // Parse comma-separated integers
        char *token = strtok(rx_buffer, ",");
        int idx = 1;
        while (token != NULL) {
          int value = atoi(token); // Or use atof() for float
          ESP_LOGI(TAG, "Value %d: %d", idx++, value);
          token = strtok(NULL, ",");
        }
      }
    }

    shutdown(sock, 0);
    close(sock);
  }

  close(listen_sock);
  vTaskDelete(NULL);
}
