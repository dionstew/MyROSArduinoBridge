/*************
  MACROS CONFIGURATIONS
 ************* */
#ifndef CONFIG_H
#define CONFIG_H

#define USE_BASE
//#undef USE_BASE

#define PID_RATE            30
#define AUTO_STOP_INTERVAL  2000

#define MAX_PWM           255
#define BAUDRATE          57600

#define CUSTOM_ELECTRIC_BIKE_DRIVER   // Define motor driver configuration
#define MEGA_I2C_NANO_ENC_COUNTER     // Define encoder configuration
#define ADDR_NANO_LEFT    0x11        // Define I2C address for each arduino nano that do the speed counting
#define ADDR_NANO_RIGHT   0x10        // Define I2C address for each arduino nano that do the speed counting
//
// #define L298_MOTOR_CUSTOM               // Define motor driver configuration
// #define ENCODER_DISK_20PPR              // Define encoder type
//#define USE_DIR_PINS
#undef USE_DIR_PINS
#undef ENCODER_DISK_20PPR

// #define USE_SERVOS                 // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS                     // Disable use of PWM servos

// define the control method
#define DIFFDRIVE_CONTROLLER          // Define diffdrive_controller as base controller
#undef CAMDRIVE_CONTROLLER            // Define camdrive_controller as base controller

//long AUTO_STOP_INTERVAL = 20000;
#define DEBUG

//========== TIMING ANALYSIS / TRANSACTION INSTRUMENTATION ==========//
// Status codes returned in transaction feedback.
#define STATUS_NORMAL             0
#define STATUS_COMMAND_TIMEOUT    1
#define STATUS_INVALID_COMMAND    2
#define STATUS_ENCODER_ERROR      3
#define STATUS_MOTOR_LIMIT_ACTIVE 4
#define STATUS_EMERGENCY_STOP     5

#endif
