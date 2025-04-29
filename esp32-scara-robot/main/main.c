#include "motor.h"
#include "motor_task.h"

motor_t motor_x = {
    .id = MOTOR_X,
    .gpio_dir = GPIOXDIR,
    .gpio_stp = GPIOXSTP,
    .target_steps = 200,
};

motor_t motor_y = {
    .id = MOTOR_Y,
    .gpio_dir = GPIOYDIR,
    .gpio_stp = GPIOYSTP,
    .target_steps = 200,
};

void app_main(void) {

  // Initialization of components!
  motor_initialization(&motor_x);
  motor_initialization(&motor_y);

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
    ;
    ;
  }
}
