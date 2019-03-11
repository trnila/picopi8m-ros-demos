#include <stdarg.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_pwm.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "motors.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Byte.h"
#include "carmotor_msgs/CarMotor.h"

ros::NodeHandle nh;

void log(const char *fmt, ...) {
  char buffer[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  nh.logerror(buffer);
  va_end(args);
}

bool is_valid_servo_val(uint16_t val) {
  return val >= 500 && val <= 2500;
}

bool is_valid_motor_val(uint8_t val) {
  return val >= 0 && val <= 100;
} 

void ros_set_servo(uint16_t val, char name, void (*setcb)(uint16_t)) {
  if(!is_valid_servo_val(val)) {
    log("Invalid value for servo%c: %d", name, val);
  } else {
    setcb(val);
  }
}

void ros_set_motor(uint8_t val, char name, void (*setcb)(uint8_t)) {
  if(val < 0 || val > 100) {
    log("Invalid value for motor%c: %d", name, val);
  } else {
    setcb(val);
  }
}

void set(const carmotor_msgs::CarMotor& state) {
  if(!is_valid_servo_val(state.servoA)) {
    log("Invalid value for servoA: %d", state.servoA);
    return;    
  }

  if(!is_valid_servo_val(state.servoB)) {
    log("Invalid value for servoB: %d", state.servoB);
    return;    
  }

  if(!is_valid_motor_val(state.motorA)) {
    log("Invalid value for motorA: %d", state.motorA);
    return;    
  }

  if(!is_valid_motor_val(state.motorB)) {
    log("Invalid value for motorB: %d", state.motorB);
    return;    
  }
  
  servoA_set(state.servoA);
  servoB_set(state.servoB);

  motorA_set(state.motorA);
  motorB_set(state.motorB);

  log("%d %d %d %d", state.servoA, state.servoB, state.motorA, state.motorB);
}

ros::Subscriber<std_msgs::UInt16> servo_a("/servo/A", [](const std_msgs::UInt16& val) {ros_set_servo(val.data, 'A', servoA_set);});
ros::Subscriber<std_msgs::UInt16> servo_b("/servo/B", [](const std_msgs::UInt16& val) {ros_set_servo(val.data, 'B', servoB_set);});

ros::Subscriber<std_msgs::Byte> motor_a("/motor/A", [](const std_msgs::Byte& val) {ros_set_motor(val.data, 'A', motorA_set);});
ros::Subscriber<std_msgs::Byte> motor_b("/motor/B", [](const std_msgs::Byte& val) {ros_set_motor(val.data, 'B', motorB_set);});

ros::Subscriber<carmotor_msgs::CarMotor> motors_all("/motors", &set);


void app_task(void *param) {
    nh.initNode();
    nh.subscribe(servo_a);
    nh.subscribe(servo_b);
    nh.subscribe(motor_a);
    nh.subscribe(motor_b);
    nh.subscribe(motors_all);

    for(;;) {
        nh.spinOnce();
    }
}

int main(void) {
	BOARD_RdcInit();
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_InitMemory();

	servo_start();
	motor_start();

  if (xTaskCreate(app_task, "APP_TASK", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
      printf("\r\nFailed to create application task\r\n"); 
      for(;;);
  }

  vTaskStartScheduler();
  printf("Failed to start FreeRTOS on core\n");
  for(;;);
}
