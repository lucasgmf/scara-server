#include "scara_controller.h"

switch_t switch_1 = {
    .value = false,
    .gpio_port = GPIO_NUM_4,
};

gpio_config_t switch_1_io_conf = {
    /* .pin_bit_mask = (1ULL << switch_1->gpio_port), */
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE // No interrupts for now
};

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t as5600_dev_handle;

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = TEST_I2C_PORT, // I2C_NUM_0
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AS5600_I2C_ADDR,
    .scl_speed_hz = 100000,
};

mag_encoder encoder_1 = {
    .raw_val = 0,
    .offset = 0,
    .is_calibrated = false,
    .cal_switch = &switch_1,
};

motor_t motor_x = {
    .id = MOTOR_X,
    .gpio_dir = GPIOXDIR,
    .gpio_stp = GPIOXSTP,
    .delay_us = 600,
};

motor_t motor_y = {
    .id = MOTOR_Y,
    .gpio_dir = GPIOYDIR,
    .gpio_stp = GPIOYSTP,
    .delay_us = 600,
};

motor_t motor_z = {
    .id = MOTOR_Z,
    .gpio_dir = GPIOZDIR,
    .gpio_stp = GPIOZSTP,
    .delay_us = 500,
};

#define PWM_FREQUENCY 500   // 500 hz
#define PWM_DUTY_CYCLE 50.0 // 50% duty cycle

#include "driver/mcpwm.h"

void init_scara() {
  /* init_switch(&switch_1, &switch_1_io_conf); */
  /* init_i2c_master(&i2c_mst_config, &dev_cfg, &bus_handle,
   * &as5600_dev_handle); */
  /* init_motor(&motor_x); */
  /* init_motor(&motor_y); */
  /* init_motor(&motor_z); */
  // Initialize MCPWM unit 0, timer 0, operator A

  gpio_reset_pin(motor_x.gpio_dir);
  gpio_set_direction(motor_x.gpio_dir, GPIO_MODE_OUTPUT);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motor_x.gpio_stp);

  // Configure the MCPWM parameters
  mcpwm_config_t pwm_config = {
      .frequency = PWM_FREQUENCY,       // 100 kHz
      .cmpr_a = PWM_DUTY_CYCLE,         // Duty cycle of PWMxA
      .cmpr_b = 0.0,                    // PWMxB not used
      .counter_mode = MCPWM_UP_COUNTER, // Count up
      .duty_mode = MCPWM_DUTY_MODE_0,   // Active HIGH
  };
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  ESP_LOGI("MCPWM", "Frequency of %d PWM started on GPIO %d", PWM_FREQUENCY,
           motor_x.gpio_stp);
  return;
}

void change_pwm_frequency(uint32_t new_freq_hz, float current_duty_percent) {
  mcpwm_config_t pwm_config = {
      .frequency = new_freq_hz,
      .cmpr_a = current_duty_percent,
      .cmpr_b = 0.0,
      .counter_mode = MCPWM_UP_COUNTER,
      .duty_mode = MCPWM_DUTY_MODE_0,
  };

  // This will briefly stop and restart the PWM output
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void loop_scara() {
  while (true) {
    change_pwm_frequency(500, 50.0);
    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    change_pwm_frequency(600, 50.0);
    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    change_pwm_frequency(700, 50.0);
    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    change_pwm_frequency(800, 50.0);
    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    change_pwm_frequency(900, 50.0);
    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    change_pwm_frequency(1000, 50.0);
    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return;
}
