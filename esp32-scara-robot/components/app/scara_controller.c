#include "scara_controller.h"

static const char *TAG = "scara_controller";

//////////////////////////////////
////// network/wifi_manager //////
//////////////////////////////////

wifi_config_t wifi_config_a = {
    .sta =
        {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
};

wifi_rec_data wifi_received_data = {
    .Kp = 1.2f,
    .Ki = 0.001f,
    .Kd = 0.1f,
    .target_position_1 = 2000,
    .target_position_2 = 2000,
};

network_configuration esp_net_conf = {
    .wifi_config = &wifi_config_a,
    .port = PORT,
    .retry_num = 0,
    .rx_buffer_size = RX_BUFFER_SIZE,
    .addr_str_size = ADDR_STR_SIZE,
    .rec_data = &wifi_received_data,
    .s_wifi_event_group = (EventGroupHandle_t)&wifi_received_data,
};

void wifi_initialization_func() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  esp_err_t wifi_result = init_wifi(&esp_net_conf);
  if (wifi_result != ESP_OK) {
    ESP_LOGE(TAG, "Exiting: Wi-Fi connection failed.");
    esp_deep_sleep_start();
    return;
  }
  xTaskCreate(tcp_server_task, "tcp_server", 4096, &esp_net_conf, 5, NULL);
  return;
}

//////////////////////////////////
////// drivers/i2c_bus ///////////
//////////////////////////////////

static SemaphoreHandle_t i2c_mutex;

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
    .i2c_mutex = &i2c_mutex,
    .timeout_ticks = portMAX_DELAY,
};

static i2c_master_dev_handle_t encoder_0_handle;
static i2c_master_dev_handle_t encoder_1_handle;
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
    .offset = 0,
    .reverse = false,
};

static encoder_t encoder_1 = {
    .label = "Encoder 1",
    .i2c_master = &i2c_master_conf,
    .i2c_slave = &i2c_slave_conf_encoder_1,
    .i2c_tca = &i2c_slave_conf_tca,
    .tca_channel = 1,
    .reg_angle_msb = ENCODER_MSB_ANGLE_REG,
    .reg_angle_mask = ENCODER_ANGLE_MASK,
    .offset = 0,
    .reverse = false,
};

void encoder_initialization_task() {
  i2c_mutex = xSemaphoreCreateMutex();

  ESP_ERROR_CHECK(
      i2c_new_master_bus(i2c_master_conf.bus_cfg, i2c_master_conf.bus_handle));

  ESP_ERROR_CHECK(i2c_master_bus_add_device(*i2c_master_conf.bus_handle,
                                            i2c_slave_conf_tca.dev_cfg,
                                            i2c_slave_conf_tca.dev_handle));

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_0.tca_channel, encoder_0.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_0.dev_cfg,
      i2c_slave_conf_encoder_0.dev_handle));

  ESP_ERROR_CHECK(encoder_init(&encoder_0));
  xTaskCreate(encoder_task, "encoder0_task", 4096, &encoder_0, 5, NULL);

  ESP_ERROR_CHECK(
      tca_select_channel(encoder_1.tca_channel, encoder_1.i2c_tca->dev_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      *i2c_master_conf.bus_handle, i2c_slave_conf_encoder_1.dev_cfg,
      i2c_slave_conf_encoder_1.dev_handle));

  ESP_ERROR_CHECK(encoder_init(&encoder_1));

  xTaskCreate(encoder_task, "encoder1_task", 4096, &encoder_1, 5, NULL);
}

//////////////////////////////////
////// hal_dir/motor /////////////
//////////////////////////////////

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

typedef struct {
  int id;
  gpio_num_t gpio_stp;
  gpio_num_t gpio_dir;
  int step_count;
  int mcpwm_unit;
  int mcpwm_timer;
  int mcpwm_opr;
  int move_ms;
  float current_freq_hz;
  int target_freq_hz;
  float speed_hz; // Hz per second
  mcpwm_timer_handle_t pwm_timer;
  mcpwm_oper_handle_t pwm_oper;
  mcpwm_cmpr_handle_t pwm_comparator;
  mcpwm_gen_handle_t pwm_generator;
  esp_timer_handle_t update_timer;
} motor_t;

void motor_init_dir(motor_t *motor_n);
void motor_create_pwm(motor_t *motor);
void motor_set_frequency(motor_t *motor, int target_freq_hz);
void motor_delete_pwm(motor_t *motor);
#define PWM_RESOLUTION_HZ 1000000

void motor_init_dir(motor_t *motor_n) {
  if (motor_n == NULL) {
    ESP_LOGW(TAG, "Pointer is null in motor_init_dir\n");
    return;
  }
  gpio_reset_pin(motor_n->gpio_dir);
  gpio_set_direction(motor_n->gpio_dir, GPIO_MODE_OUTPUT);
  ESP_LOGW(TAG, "Successfully initializated motor %d", motor_n->id);

  return;
}

#define PWM_RESOLUTION_HZ 1000000 // 1 MHz
#define MCPWM_MIN_PERIOD_TICKS 5
#define PWM_FREQ_MIN 1.0f
#define PWM_FREQ_MAX ((float)PWM_RESOLUTION_HZ / (float)MCPWM_MIN_PERIOD_TICKS)
#define MCPWM_MAX_PERIOD_TICKS                                                 \
  60000 // Add a safety margin
        //
void motor_create_pwm(motor_t *motor) {
  if (!motor) {
    ESP_LOGE(TAG, "motor pointer is NULL");
    return;
  }

  motor_delete_pwm(motor);

  // Validate frequency
  if (motor->current_freq_hz < PWM_FREQ_MIN) {
    ESP_LOGW(TAG, "Skipping PWM creation: freq = %.2f is too low",
             motor->current_freq_hz);
    return;
  }

  // Clamp frequency to a safe range
  float freq = motor->current_freq_hz;
  if (freq > PWM_FREQ_MAX)
    freq = PWM_FREQ_MAX;
  if (freq < PWM_FREQ_MIN)
    freq = PWM_FREQ_MIN;

  uint32_t period_ticks = (uint32_t)(PWM_RESOLUTION_HZ / freq);
  if (period_ticks < MCPWM_MIN_PERIOD_TICKS)
    period_ticks = MCPWM_MIN_PERIOD_TICKS;
  if (period_ticks > MCPWM_MAX_PERIOD_TICKS)
    period_ticks = MCPWM_MAX_PERIOD_TICKS;

  uint32_t duty_ticks = period_ticks / 2;

  // Create timer
  mcpwm_timer_config_t timer_config = {
      .group_id = motor->mcpwm_unit,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = PWM_RESOLUTION_HZ,
      .period_ticks = period_ticks,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };

  esp_err_t err;
  err = mcpwm_new_timer(&timer_config, &motor->pwm_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(err));
    return;
  }

  // Create operator and connect timer
  mcpwm_operator_config_t operator_config = {.group_id = motor->mcpwm_unit};
  err = mcpwm_new_operator(&operator_config, &motor->pwm_oper);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM operator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_operator_connect_timer(motor->pwm_oper, motor->pwm_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to connect operator to timer: %s",
             esp_err_to_name(err));
    return;
  }

  // Create comparator
  mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez =
                                                     true};
  err = mcpwm_new_comparator(motor->pwm_oper, &comparator_config,
                             &motor->pwm_comparator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create comparator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_comparator_set_compare_value(motor->pwm_comparator, duty_ticks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set compare value: %s", esp_err_to_name(err));
    return;
  }

  // Create generator
  mcpwm_generator_config_t generator_config = {.gen_gpio_num = motor->gpio_stp};
  err = mcpwm_new_generator(motor->pwm_oper, &generator_config,
                            &motor->pwm_generator);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create generator: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_timer_event(
      motor->pwm_generator,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY,
                                   MCPWM_GEN_ACTION_HIGH));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator high action: %s",
             esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_compare_event(
      motor->pwm_generator, MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, motor->pwm_comparator,
                                MCPWM_GEN_ACTION_LOW));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set generator low action: %s",
             esp_err_to_name(err));
    return;
  }

  // Enable and start timer
  err = mcpwm_timer_enable(motor->pwm_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(err));
    return;
  }

  err = mcpwm_timer_start_stop(motor->pwm_timer, MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    return;
  }

  ESP_LOGI(TAG, "PWM created for motor %d at freq %.2f Hz (period %u ticks)",
           motor->id, freq, period_ticks);
}

void motor_delete_pwm(motor_t *motor) {
  if (!motor)
    return;

  if (motor->pwm_timer) {
    mcpwm_timer_disable(motor->pwm_timer); // Safe to disable before delete
    mcpwm_del_timer(motor->pwm_timer);
    motor->pwm_timer = NULL;
  }

  if (motor->pwm_generator) {
    mcpwm_del_generator(motor->pwm_generator);
    motor->pwm_generator = NULL;
  }

  if (motor->pwm_comparator) {
    mcpwm_del_comparator(motor->pwm_comparator);
    motor->pwm_comparator = NULL;
  }

  if (motor->pwm_oper) {
    mcpwm_del_operator(motor->pwm_oper);
    motor->pwm_oper = NULL;
  }
}

#define UPDATE_INTERVAL_MS 100 // timer period

void motor_update_timer_cb(void *arg) {
  motor_t *motor = (motor_t *)arg;

  int steps_this_period =
      (int)(motor->current_freq_hz * UPDATE_INTERVAL_MS / 1000.0f);

  // Accumulate steps
  motor->step_count += steps_this_period;

  float step = motor->speed_hz * 0.1f;
  float diff = motor->target_freq_hz - motor->current_freq_hz;

  if (fabs(diff) < step) {
    motor->current_freq_hz = motor->target_freq_hz;
    esp_timer_stop(motor->update_timer);
  } else {
    motor->current_freq_hz += (diff > 0 ? step : -step);
  }

  motor_delete_pwm(motor);

  if (motor->current_freq_hz > 0.0f) {
    motor_create_pwm(motor); // Only restart if freq > 0
  } else {
    ESP_LOGI("motor", "PWM stopped at 0 Hz");
  }
}

void motor_set_frequency(motor_t *motor, int target_freq_hz) {
  if (motor->target_freq_hz == target_freq_hz) {
    return;
  }

  motor->target_freq_hz = target_freq_hz;

  if (motor->update_timer == NULL) {
    const esp_timer_create_args_t timer_args = {.callback =
                                                    &motor_update_timer_cb,
                                                .arg = motor,
                                                .name = "motor_freq_updater"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motor->update_timer));
  }

  if (!esp_timer_is_active(motor->update_timer)) {
    ESP_ERROR_CHECK(
        esp_timer_start_periodic(motor->update_timer, 100000)); // 100ms
  }
}

typedef struct {
  /* encoder_t *encoder; */
  motor_t *motor;
  pid_controller_t *pid;
  float target_position;    // Desired encoder position
  float current_position;   // Read from encoder
  float output_freq_hz;     // Output to motor (used as PWM freq)
  float max_output_freq_hz; // Safety clamp
  bool direction_reversed;  // Optional motor direction flip
} motor_control_loop_t;

motor_t motor_x = {
    .id = 0,
    .gpio_stp = GPIOXSTP,
    .gpio_dir = GPIOXDIR,
    .step_count = 0,
    .mcpwm_unit = 0,
    .mcpwm_timer = 0,
    .mcpwm_opr = 0,
    .move_ms = 0,
    .current_freq_hz = 200,
    .target_freq_hz = 0,
    .speed_hz = 800,
};

motor_t motor_y = {
    .id = 1,
    .gpio_stp = GPIOYSTP,
    .gpio_dir = GPIOYDIR,
    .step_count = 0,
    .mcpwm_unit = 1,
    .mcpwm_timer = 0,
    .mcpwm_opr = 0,
    .move_ms = 0,
    .current_freq_hz = 200,
    .target_freq_hz = 0,
    .speed_hz = 800, // variation speed
};

static pid_controller_t pid1 = {
    .Kp = 1.2f,
    .Ki = 0.001f,
    .Kd = 0.1f,
    .output_limit = 500.0f,
};

static motor_control_loop_t loop1 = {
    /* .encoder = &encoder1, */
    NULL,
    /* .motor = &motor_x, */
    .pid = &pid1,
    .target_position = 1200.0f,
    .max_output_freq_hz = 1100.0f,
    .direction_reversed = false,
};

void motor_initialization_task() {
  motor_init_dir(&motor_x);
  motor_create_pwm(&motor_x);

  xTaskCreate(motor_control_task, "motor_ctrl", 4096, &loop1, 5, NULL);
  return;
}

void init_scara() {
  /* wifi_initialization_func(); */
  encoder_initialization_task();
  /* motor_initialization_task(); */

  ESP_LOGI(TAG, "");
  return;
}

void loop_scara() {
  while (1) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

  return;
}
