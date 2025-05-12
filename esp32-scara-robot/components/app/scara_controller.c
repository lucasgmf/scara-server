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

void init_scara() {
  /* init_switch(&switch_1, &switch_1_io_conf); */
  /* init_i2c_master(&i2c_mst_config, &dev_cfg, &bus_handle,
   * &as5600_dev_handle); */
  /* init_motor(&motor_x); */
  /* init_motor(&motor_y); */
  /* init_motor(&motor_z); */
  // Initialize MCPWM unit 0, timer 0, operator A

  init_motor_dir(&motor_x);
  init_motor_stp(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor_x.gpio_stp);

  ESP_LOGI("init_scara", "init_scara ended without errors!");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  return;
}
void loop_scara() {

  while (true) {
    apply_motor_pwm(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0, 500);

    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    apply_motor_pwm(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0, 600);
    ESP_LOGI("loop_scara", "Changing direction to 1");
    gpio_set_level(motor_x.gpio_dir, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("loop_scara", "Changing direction to 0");
    gpio_set_level(motor_x.gpio_dir, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return;
}
