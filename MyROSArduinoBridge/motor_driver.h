/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
   
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#if defined(L298_MOTOR_DRIVER)
  #define RIGHT_MOTOR_BACKWARD  5
  #define LEFT_MOTOR_BACKWARD   6
  #define RIGHT_MOTOR_FORWARD   9
  #define LEFT_MOTOR_FORWARD    10
  #define RIGHT_MOTOR_ENABLE    12
  #define LEFT_MOTOR_ENABLE     13

#elif defined(CUSTOM_ELECTRIC_BIKE_DRIVER)
  #define RIGHT_MOTOR_PWM_PIN       7   // TODO: change to a correct Pin Number
  #define LEFT_MOTOR_PWM_PIN        6   // TODO: change to a correct Pin Number
  #define RIGHT_DIRECTION_RELAY_PIN 37  // TODO: change to a correct Pin Number
  #define LEFT_DIRECTION_RELAY_PIN  39  // TODO: change to a correct Pin Number
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void setMotorSpeed_PID(int i, int spd);
void setMotorSpeeds_PID(long leftSpeed, long rightSpeed);

#endif
