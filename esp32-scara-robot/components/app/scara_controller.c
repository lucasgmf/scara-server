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
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_0,
    .mcpwm_opr = MCPWM_OPR_A,
    .move_ms = 2000,
    .freq_hz = 700,
};

motor_t motor_y = {
    .id = MOTOR_Y,
    .gpio_dir = GPIOYDIR,
    .gpio_stp = GPIOYSTP,
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_1,
    .mcpwm_opr = MCPWM_OPR_A,
    .move_ms = 1000,
    .freq_hz = 700,
};

motor_t motor_z = {
    .id = MOTOR_Z,
    .gpio_dir = GPIOZDIR,
    .gpio_stp = GPIOZSTP,
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_2,
    .mcpwm_opr = MCPWM_OPR_A,
    .move_ms = 1500,
    .freq_hz = 1000,
};

void init_scara() {
  init_motor_dir(&motor_x);
  init_motor_stp(&motor_x);

  init_motor_dir(&motor_y);
  init_motor_stp(&motor_y);

  /* init_motor_dir(&motor_z); */
  /* init_motor_stp(&motor_z); */

  ESP_LOGI("init_scara", "init_scara ended without errors!");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  return;
}
void loop_scara() {
  xTaskCreate(move_test_motor, "motor_x_task", 2048, (void *)&motor_x, 10,
              NULL);
  xTaskCreate(move_test_motor, "motor_y_task", 2048, (void *)&motor_y, 10,
              NULL);

  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return;
}
