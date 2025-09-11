/*************
  MACROS CONFIGURATIONS
 ************* */
#ifndef CONFIG_H
#define CONFIG_H

#define USE_BASE
#define ADDR_NANO_LEFT    0x11
#define ADDR_NANO_RIGHT   0x10

#define PID_RATE            30
#define AUTO_STOP_INTERVAL  5000

#define MAX_PWM           255
#define BAUDRATE          57600

#define CUSTOM_ELECTRIC_BIKE_DRIVER  // Define motor driver configuration
#define MEGA_I2C_NANO_ENC_COUNTER  // Define encoder configuration

// #define USE_SERVOS              // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS  // Disable use of PWM servos

#endif