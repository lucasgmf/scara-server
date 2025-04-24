#include "motor.h"
#include "motor_task.h"

motor_t motor_x = {
    .id = MOTOR_X,
    .gpio_dir = GPIOXDIR,
    .gpio_stp = GPIOXSTP,
};

motor_t motor_y = {
    .id = MOTOR_Y,
    .gpio_dir = GPIOYDIR,
    .gpio_stp = GPIOYSTP,
};

void app_main(void) {
  xTaskCreatePinnedToCore(motor_task,     // task function
                          "Motor_X_task", // name
                          2048,           // stack size
                          &motor_x,       // task parameters
                          5,    //  priority  (higher means higher priority)
                          NULL, // handle
                          1     // core
  );
  xTaskCreatePinnedToCore(motor_task, // task function
                          "Motor_Y",  // name
                          2048,       // stack size
                          &motor_y,   // task parameters
                          4,    //  priority  (higher means higher priority)
                          NULL, // handle
                          1     // core
  );

  while (1) {

    ESP_LOGI("Main", "Hello from main!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
