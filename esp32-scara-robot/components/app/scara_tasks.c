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

#define MAX_ENCODER_VAL 4096 // 12-bit encoder
#define HALF_ENCODER_VAL (MAX_ENCODER_VAL / 2)

void encoder_task(void *arg) {
  encoder_t *encoder = (encoder_t *)arg;
  if (encoder == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  // Configuration validation
  uint16_t max_encoder_val = encoder->encoder_resolution;
  int16_t half_encoder_val = max_encoder_val / 2;

  ESP_LOGI(encoder->label,
           "Starting encoder task - resolution: %d, gear ratio: %.2f",
           max_encoder_val, encoder->gear_ratio);

  // First read to establish baseline
  uint16_t last_angle = encoder_read_angle(encoder);
  if (last_angle == 0xFFFF) {
    ESP_LOGE(encoder->label, "Failed initial encoder read");
    vTaskDelete(NULL);
    return;
  }

  uint16_t current_angle;
  int16_t delta;
  uint32_t error_count = 0;

  while (1) {
    current_angle = encoder_read_angle(encoder);

    if (current_angle == 0xFFFF) {
      error_count++;
      ESP_LOGW(encoder->label, "Failed to read angle (error count: %lu)",
               error_count);

      // If too many consecutive errors, consider resetting or taking action
      if (error_count > 10) {
        ESP_LOGE(encoder->label,
                 "Too many consecutive read errors, continuing...");
        error_count = 0; // Reset counter
      }

      vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay on error
      continue;
    }

    error_count = 0; // Reset error counter on successful read

    // Calculate delta with wraparound handling
    delta = (int16_t)current_angle - (int16_t)last_angle;

    // Handle wraparound (shortest path)
    if (delta > half_encoder_val) {
      delta -= max_encoder_val;
    } else if (delta < -half_encoder_val) {
      delta += max_encoder_val;
    }

    // Update accumulated steps
    encoder->accumulated_steps += delta;

    // Update angle calculations
    encoder_update_absolute_angle(encoder);

    // Optional: Log current position periodically
    /* static uint32_t log_counter = 0; */
    /* if (++log_counter % 100 == 0) { // Log every 5 seconds at 50ms intervals */
    /*   ESP_LOGI(encoder->label, */
    /*            "Raw: %d, Steps: %ld, Motor: %.2f°, Output: %.2f°", */
    /*            current_angle, encoder->accumulated_steps, */
    /*            encoder->motor_angle_degrees, encoder->angle_degrees); */
    /* } */
    /**/
    last_angle = current_angle;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
/* ESP_LOGI(encoder->label, "Raw angle: %u", current_angle); */
/* ESP_LOGI(encoder->label, */
/*          "Angle: %u | Delta: %d | Position: %ld | angle_deg: %.2f | " */
/*          "angle_rad :%.2f", */
/*          current_angle, delta, encoder->accumulated_steps, */
/*          encoder->accumulated_steps * 360.0 / 4096 / encoder->gear_ratio,
 */
/*          encoder->accumulated_steps * 2 * M_PI / encoder->gear_ratio / */
/*              4096); */

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

    if (!motor->control_vars->ref_encoder->is_calibrated) {
    /* ESP_LOGI("motor_control_task", "encoder is not calibrated"); */
      vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
      continue;
    }

    error = motor->control_vars->encoder_target_pos -
            motor->control_vars->ref_encoder->accumulated_steps;

    const float DEAD_BAND_THRESHOLD = 2.0f;            // WARN: define this
    float dt = MOTOR_CONTROL_TASK_PERIOD_MS / 1000.0f; // Time in seconds

    // Deadband: stop motor and prevent integral windup
    if (fabsf(error) < DEAD_BAND_THRESHOLD) {
      motor->control_vars->pid->integral = 0.0f;
      motor->pwm_vars->target_freq_hz = 0.0f;
      motor_set_target_frequency(motor, 0.0f);
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

    if (true) {
      ESP_LOGI("PID",
               "error: %.2f - output: %.2f | "
               "Kp*error = %.2f, Ki* integral = %.2f, Kd * derivative = %.2f",

               error, output, motor->control_vars->pid->Kp * error,
               motor->control_vars->pid->Ki *
                   motor->control_vars->pid->integral,
               motor->control_vars->pid->Kd * derivative);
    }

    // Clamp output to allowed frequency
    if (output > motor->pwm_vars->max_freq)
      output = motor->pwm_vars->max_freq;
    if (output < -motor->pwm_vars->max_freq)
      output = -motor->pwm_vars->max_freq;

    // WARN: maybe this is target_freq_hz??
    /* loop->output_freq_hz = fabsf(output); */

    /* ESP_LOGI("motor_control_task", "saving target freq to %f",
     * fabs(output));
     */
    local_target_freq_hz = fabsf(output);

    // Determine direction
    bool reverse = output < 0;

    // TODO: If direction is the same as previous, do not set level again...
    /* ESP_LOGW("debug", "setting dir to %d", */
    /*          motor->pwm_vars->dir_is_reversed ? !reverse : !reverse); */

    gpio_set_level(motor->gpio_dir,
                   motor->pwm_vars->dir_is_reversed ? !reverse : reverse);

    // Apply new frequency to motor
    motor_set_target_frequency(motor, local_target_freq_hz);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
  }
}

void switch_task(void *arg) {
  switch_t *switch_n = (switch_t *)arg;
  if (switch_n == NULL) {
    ESP_LOGE(TAG, "Parameter is null, aborting.");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    update_switch_val(switch_n);
    /* ESP_LOGI("switch_task", "value of switch_n is %d", switch_n->is_pressed);
     */
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}
