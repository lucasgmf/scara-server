#include "scara_tasks.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char *TAG = "scara_tasks";

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
  if (handle_error(listen_sock < 0, "Unable to create socket", -1))
    return;
  ESP_LOGI(TAG, "Socket created");

  int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (handle_error(err != 0, "Socket bind failed", listen_sock))
    return;
  ESP_LOGI(TAG, "Socket bound, port %d", net_conf->port);

  err = listen(listen_sock, 1);
  if (handle_error(err != 0, "Listen failed", listen_sock))
    return;

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
      ESP_LOGI(TAG, "Free stack: %d", uxTaskGetStackHighWaterMark(NULL));
      int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
      if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        break;
      } else if (len == 0) {
        ESP_LOGI(TAG, "Connection closed");
        break;
      }

      else {
        rx_buffer[len] = 0;
        ESP_LOGI(TAG, "Received: %s", rx_buffer);

        int count = sscanf(rx_buffer, "%f,%f,%f,%d,%d", &net_conf->rec_data->Kp,
                           &net_conf->rec_data->Ki, &net_conf->rec_data->Kd,
                           &net_conf->rec_data->target_position_1,
                           &net_conf->rec_data->target_position_2);

        if (count == 5) {
          ESP_LOGI(TAG, "Parsed values:");
          ESP_LOGI(TAG, "Kp: %.2f, Ki: %.2f, Kd: %.2f", net_conf->rec_data->Kp,
                   net_conf->rec_data->Ki, net_conf->rec_data->Kd);
          ESP_LOGI(TAG, "Target: %.2f, Test Num: %.2f",
                   net_conf->rec_data->target_position_1,
                   net_conf->rec_data->target_position_2);

          // Use parsed.Kp, parsed.Ki, etc. as needed in your control logic
        } else {
          ESP_LOGW(TAG, "Failed to parse input data. Received: %s", rx_buffer);
        }
      }
    }

    shutdown(sock, 0);
    close(sock);
  }

  close(listen_sock);
  vTaskDelete(NULL);
}

void encoder_task(void *arg) {
  encoder_t *encoder = (encoder_t *)arg;
  if (encoder == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    uint16_t angle = encoder_read_angle(encoder);

    if (angle != 0xFFFF) {
      /* ESP_LOGI(encoder->label, "Angle: %u", angle); */
    } else {
      ESP_LOGW(encoder->label, "Failed to read angle");
    }

    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

#define TASK_UPDATE_MOTOR_PWM_PERIOD_MS 20

/* void task_update_motor_pwm(void *arg) { */
/*   motor_t *motor_n = (motor_t *)arg; */
/*   if (motor_n == NULL) { */
/*     ESP_LOGE(TAG, "Parameter is null, aborting."); */
/*     vTaskDelete(NULL); */
/*     return; */
/*   } */
/**/
/*   TickType_t last_wake_time = xTaskGetTickCount(); */
/*   const float dt_sec = TASK_UPDATE_MOTOR_PWM_PERIOD_MS / 1000.0f; */
/**/
/*   while (true) { */
/*     ESP_LOGI(TAG, "Current freq: %.2f Hz, Target: %d Hz", */
/*              motor_n->current_freq_hz, motor_n->target_freq_hz); */
/**/
/*     float diff = motor_n->target_freq_hz - motor_n->current_freq_hz; */
/*     float step = motor_n->speed_hz * dt_sec; */
/**/
/*     // Ramp frequency toward target */
/*     if (fabsf(diff) < step) { */
/*       motor_n->current_freq_hz = motor_n->target_freq_hz; */
/*     } else { */
/*       motor_n->current_freq_hz += (diff > 0) ? step : -step; */
/*     } */
/**/
/*     // Update step count estimate */
/*     motor_n->step_count += (int)(motor_n->current_freq_hz * dt_sec); */
/**/
/*     // Update PWM */
/*     motor_delete_pwm(motor_n); */
/*     if (motor_n->current_freq_hz > 0.0f) { */
/*       motor_create_pwm(motor_n); */
/*     } */
/**/
/*     // Wait for next tick */
/*     vTaskDelayUntil(&last_wake_time, */
/*                     pdMS_TO_TICKS(TASK_UPDATE_MOTOR_PWM_PERIOD_MS)); */
/*   } */
/* } */

#define MOTOR_CONTROL_TASK_PERIOD_MS 20

void motor_control_task(void *arg) {
  motor_t *motor = (motor_t *)arg;
  if (motor == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  float error = 0;
  float output = 0;
  float local_target_freq_hz = 0;

  while (1) {
    error = motor->control_vars->encoder_target_pos -
            motor->control_vars->ref_encoder->current_reading;

    const float DEAD_BAND_THRESHOLD = 2.0f;            // WARN: define this
    float dt = MOTOR_CONTROL_TASK_PERIOD_MS / 1000.0f; // Time in seconds

    // Deadband: stop motor and prevent integral windup
    if (fabsf(error) < DEAD_BAND_THRESHOLD) {
      motor->control_vars->pid->integral = 0.0f;
      motor->pwm_vars->target_freq_hz = 0.0f;
      motor_set_frequency(motor, 0.0f);
      vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
      continue;
    }

    // PID calculations
    motor->control_vars->pid->integral += error * dt;

    // Optional: limit the integral term to prevent windup
    const float INTEGRAL_LIMIT = 2000.0f * 4;
    if (motor->control_vars->pid->integral > INTEGRAL_LIMIT)
      motor->control_vars->pid->integral = INTEGRAL_LIMIT;
    else if (motor->control_vars->pid->integral < -INTEGRAL_LIMIT)
      motor->control_vars->pid->integral = -INTEGRAL_LIMIT;

    float derivative = (error - motor->control_vars->pid->prev_error) / dt;
    motor->control_vars->pid->prev_error = error;

    output = motor->control_vars->pid->Kp * error +
             motor->control_vars->pid->Ki * motor->control_vars->pid->integral +
             motor->control_vars->pid->Kd * derivative;

    ESP_LOGI("PID",
             "error: %.2f - output: %.2f | "
             "Kp*error = %.2f, Ki* integral = %.2f, Kd * derivative = %.2f",

             error, output, motor->control_vars->pid->Kp * error,
             motor->control_vars->pid->Ki * motor->control_vars->pid->integral,
             motor->control_vars->pid->Kd * derivative);

    // Clamp output to allowed frequency
    if (output > motor->pwm_vars->max_freq)
      output = motor->pwm_vars->max_freq;
    if (output < -motor->pwm_vars->max_freq)
      output = -motor->pwm_vars->max_freq;

    // WARN: maybe this is target_freq_hz??
    /* loop->output_freq_hz = fabsf(output); */

    /* ESP_LOGI("motor_control_task", "saving target freq to %f", fabs(output));
     */
    local_target_freq_hz = fabsf(output);

    // Determine direction
    bool reverse = output < 0;
    gpio_set_level(motor->gpio_dir,
                   motor->pwm_vars->dir_is_reversed ? !reverse : reverse);

    // Apply new frequency to motor
    motor_set_frequency(motor, local_target_freq_hz);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
  }
}
