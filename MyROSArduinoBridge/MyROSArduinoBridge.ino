/*********************************************************************
 *  ROSArduinoBridge
  
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/********************************************************************
  * MyRosArduinoBridge: Inspired by RosArduinoBridge.
    Still, this version is a set of simple serial commands to control 
    a differential drive robot and receive back sensor and odometry data. 
    I only modified the main code to match my robot's configuration. 
    However, the big changes are located in encoder_driver, motor_driver
    and main code.

    Also, I tidied up a little bit so the code look a bit aesthetic. 
    Besides, another definitions are still same.    
 *********************************************************************/

/********************************************************************
  * Update        : Last edited on September 10, 2025. At 11:26 PM.
  * Case Solved   : 1. There was a bug logic. Motor would only run if the 
                    corresponding brake is turned off -> brakes ON = motors DISABLED. 
                    Make sure to remember those in the future.
  * TODO    : Check the possibility of overflowing values in encoder vars
              if the encoder runs without stops. Maybe think a framework 
              to reset encoder values.

  Have a nice day ~~~
 *********************************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//========== MACROS VARIABLES DEFINITIONS ==========//
#include "config.h"
#include "commands.h"                 // Include definition of serial commands

#ifdef USE_BASE                       // Define the motor controller and encoder library you are using
  #include "encoder_driver.h"           // Encoder driver function definitions
  #include "motor_driver.h"             // Motor driver function definitions
  #include "sensors.h"                  // Sensor functions
  #include "brakes.h"                   // Brakes functions
  
  #ifdef DIFFDRIVE_CONTROLLER
    #include "diff_controller.h"          // PID parameters and functions of diffdrive_controller
  #endif                                        // endif DIFFDRIVE_CONTROLLER
  
  #ifdef CAMDRIVE_CONTROLLER
    #include "camdrive_controller.h"      // PID parameters and functions of camdrive_controller
  #endif                                        // endif CAMDRIVE_CONTROLLER
#endif                                        // endif USE_BASE

#ifdef USE_SERVOS  // Include servo support if defined
  #include <Servo.h>
  #include "servos.h"
#endif

#ifdef MEGA_I2C_NANO_ENC_COUNTER
  EncoderData encoderLeft;
  EncoderData encoderRight;
#endif

#ifdef ENCODER_DISK_20PPR
  EncoderData encoderLeft;
  EncoderData encoderRight;
  volatile uint8_t prev_stateA = 0;  
  volatile uint8_t prev_stateB = 0;
  volatile long encoder_ticksA = 0;
  volatile long encoder_ticksB = 0;
  volatile int8_t g_left_dir_sign  = 0;  // +1 forward, -1 backward, 0 stopped/brake
  volatile int8_t g_right_dir_sign = 0;
#endif

const int PID_INTERVAL  = 1000 / PID_RATE;    // Convert the rate into an interval (time sampling)
unsigned long nextPID   = PID_INTERVAL;       // Track the next time we make a PID calculation
long lastMotorCommand   = AUTO_STOP_INTERVAL; 

/* Variable initialization */
int arg         = 0;  // A pair of varibles to help parse serial commands (thanks Fergs)
int index       = 0;
char chr;             // Variable to hold an input character
char cmd;             // Variable to hold the current single-character command
char argv1[30];       // Character arrays to hold command arguments
char argv2[30];
char argv3[30];       // Added for timing transaction: f <seq> <left> <right>
long arg1;            // The arguments converted to integers
long arg2;
long arg3;
int ang_ser = 0;
char buffer[100];

//========== TIMING ANALYSIS / TRANSACTION INSTRUMENTATION ==========//
unsigned long g_cmd_rx_us = 0;
unsigned long g_last_command_us = 0;
unsigned long g_last_target_update_us = 0;
unsigned long g_last_encoder_read_us = 0;
unsigned long g_last_feedback_tx_us = 0;
unsigned long g_last_pid_update_us = 0;

unsigned long g_cmd_rx_count = 0;
unsigned long g_feedback_tx_count = 0;
unsigned long g_timeout_count = 0;
unsigned long g_parse_error_count = 0;
unsigned long g_safe_stop_count = 0;

uint8_t g_status_code = STATUS_NORMAL;
bool g_in_timeout_safe_stop = false;

void resetCommand() {  // Clear the current command parameters
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  arg1  = 0;
  arg2  = 0;
  arg3  = 0;
  arg   = 0;
  index = 0;
}
bool hasArg1() {
  return argv1[0] != '\0';
}

bool hasArg2() {
  return argv2[0] != '\0';
}

bool hasArg3() {
  return argv3[0] != '\0';
}

bool requireArgs(uint8_t n) {
  if (n >= 1 && !hasArg1()) return false;
  if (n >= 2 && !hasArg2()) return false;
  if (n >= 3 && !hasArg3()) return false;
  return true;
}
int runCommand() {  // Run a command.  Commands are defined in commands.h
  char  *str;
  int   pid_args[4];
  
  int   i     = 0;
  char  *p    = argv1;
  arg1        = atoi(argv1);
  arg2        = atoi(argv2);
  arg3        = atoi(argv3);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;

    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;

    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;

    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;

    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;

    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;

    case PING:
      Serial.println(Ping(arg1));
      break;

    case CEK_DATA:
      Serial.println("Command received!");
      break;

    #ifdef USE_SERVOS
    case SERVO_WRITE:
      if (arg1 < 0 || arg1 >= N_SERVOS) { Serial.println("ERR"); break; }
      int pos = constrain((int)arg2, 0, 180);
      servos[arg1].setTargetPosition(pos);
      Serial.println("OK");
      break;

    case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;
    #endif

  #ifdef USE_BASE
    case READ_ENCODERS:  // Modify inside this part. Could be set a conditional statements
      #if defined(MEGA_I2C_NANO_ENC_COUNTER)  // Reading encoder data with Nano I2C configuration
        readEncoder_I2C(ADDR_NANO_LEFT,  encoderLeft);                 // Reading left encoder data with Nano I2C configuration
        readEncoder_I2C(ADDR_NANO_RIGHT, encoderRight);               // Reading right encoder data with Nano I2C configuration
        Serial.print(encoderLeft.ticks);      
        Serial.print(" ");
        Serial.println(encoderRight.ticks);
              
      #elif defined(ENCODER_DISK_20PPR)
        // readEncoder_20PPR(LEFT, encoderLeft);
        // readEncoder_20PPR(RIGHT, encoderRight);
        // Serial.print(encoderLeft.ticks);
        // Serial.print(" ");
        // Serial.println(encoderRight.ticks);
        Serial.println("cuk gagal terus");
      #else
        Serial.print(readEncoder(LEFT));      // Reading left encoder data with standard configuration
        Serial.print(" ");  
        Serial.println(readEncoder(RIGHT));   // Reading right encoder data with standard configuration
      #endif
      break;

    case RESET_ENCODERS:                    // Modify inside this part. Could be set conditional statements
      #if defined(MEGA_I2C_NANO_ENC_COUNTER)  // Reset encoder data with Nano I2C configuration
        resetEncoders_I2C();                  
        resetPID();
        Serial.println("OK");
      #elif defined(ENCODER_DISK_20PPR)
        resetEncoder_20PPR();
        Serial.println("OK");
      #else
        resetEncoders();                      // Reset encoder data with standard configuration
        resetPID();
        Serial.println("OK");
      #endif
      break;
      
    case MOTOR_SPEEDS:
      // Legacy command used by the original ROSArduinoBridge hardware interface.
      applyDiffDriveCommand(arg1, arg2);      
      Serial.println("OK");
      break;

    case TIMING_TRANSACTION:
      // New transaction command for distributed MCU timing analysis:
      // f <seq_id> <left_cmd> <right_cmd>
      // Response:
      // fb <seq_id> <cmd_rx_us> <target_update_us> <encoder_read_us> <feedback_tx_us> <left_ticks> <right_ticks> <status_code>
      g_cmd_rx_us = micros();
      g_cmd_rx_count++;
      applyDiffDriveCommand(arg2, arg3);
      sendTimingFeedback((unsigned long)arg1, arg2, arg3);
      break;
    
    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();  // Reset the auto stop timer
      g_last_command_us = micros();
      g_in_timeout_safe_stop = false;
      g_status_code = STATUS_NORMAL;
      resetPID();
      setAllBrakes(false, false);
      moving = 0;  // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2);
      g_last_target_update_us = micros();
      Serial.println("OK");
      break;
    
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0];
      Ki = pid_args[1];
      Kd = pid_args[2];
      Ko = pid_args[3];
      break;
  #endif
    
    case ALL_BRAKE_OFF:
      setAllBrakes(false, false);
      break;
      
    default:
      // g_status_code = STATUS_INVALID_COMMAND;
      // g_parse_error_count++;
      Serial.println("Invalid Command");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  Serial.println("Init...");
  // Initialize the motor controller if used */
  // Initialization Encoders
  #if defined(USE_BASE)
    #if defined(ARDUINO_ENC_COUNTER)
      initEncoder();
    #elif defined(MEGA_I2C_NANO_ENC_COUNTER)
      initEncoderInterface();
      //  Serial.println("Encoder has been setup");
    #elif defined(ENCODER_DISK_20PPR)
      initEncoderDisk();
    #endif

    initMotorController();
    initElectricalBrakes();
    resetPID();
    setAllBrakes(true, true);                     // set All brakes ON to hold position

    #if defined(DEBUG)
    Serial.println("Motor has been initialized!");
    Serial.println("Brakes has been initialized!");
    Serial.println("PID has been reset");
    #endif
  #endif
  
  #ifdef USE_SERVOS                     // Attach servos if used
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
        servoPins[i],
        stepDelay[i],
        servoInitPosition[i]);
    }
  #endif
  #if defined(DEBUG)
  Serial.println("left \t right");
  Serial.println(0);
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/

void loop() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();
    // Do not echo characters in protocol mode. Echoing corrupts ROS-side responses.
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      else if (arg == 3) argv3[index] = NULL;
      g_cmd_rx_us = micros();
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) {
        arg = 1;
        // index = 0;
      }
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      else if (arg == 2) {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }
      continue;
    } else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      } else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      } else if (arg == 2) {
        if (index < (int)sizeof(argv2) - 1) {
          argv2[index] = chr;
          index++;
        }
      } else if (arg == 3) {
        if (index < (int)sizeof(argv3) - 1) {
          argv3[index] = chr;
          index++;
        }
      }
    }
  }
  
  // If we are using base control, run a PID calculation at the appropriate intervals
  #ifdef USE_BASE  
    if (millis() > nextPID) {
      updatePID();
      g_last_pid_update_us = micros();
      // printDebugMsg();
      nextPID += PID_INTERVAL;
    }
    // Check to see if we have exceeded the auto-stop interval
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
      setMotorSpeeds(0, 0);
      moving = 0;
      setAllBrakes(true, true);
      if (!g_in_timeout_safe_stop) {
        g_timeout_count++;
        g_safe_stop_count++;
        g_status_code = STATUS_COMMAND_TIMEOUT;
        g_in_timeout_safe_stop = true;
      }
    }
  #endif

  // Sweep servos
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].doSweep();
//      servos[i].jumpSweep();
    }
  #endif
}
