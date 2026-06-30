/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#if defined(USE_BASE)
   
#if defined(POLOLU_VNH5019)
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined(POLOLU_MC33926)
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined(L298_MOTOR_DRIVER)
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined(L298_MOTOR_CUSTOM)
  void initMotorController() {
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
    {
      spd = 255;
      }
    
    if (i == LEFT) { 
      g_left_dir_sign = (spd == 0) ? 0 : (reverse ? -1 : +1);
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      g_right_dir_sign = (spd == 0) ? 0 : (reverse ? -1 : +1);
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined(CUSTOM_ELECTRIC_BIKE_DRIVER)
  
  void initMotorController() {
    // setup pin Motors and Directions as OUTPUT
    pinMode(LEFT_MOTOR_PWM_PIN,         OUTPUT);
    pinMode(LEFT_DIRECTION_RELAY_PIN,   OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN,        OUTPUT);
    pinMode(RIGHT_DIRECTION_RELAY_PIN,  OUTPUT);

    // set initial value
    analogWrite(LEFT_MOTOR_PWM_PIN,         0);
    analogWrite(RIGHT_MOTOR_PWM_PIN,        0);
    digitalWrite(LEFT_DIRECTION_RELAY_PIN,  LOW); // Logic: HIGH = Forward, LOW = Reverse (Active LOW Relay)
    digitalWrite(RIGHT_DIRECTION_RELAY_PIN, LOW); // Logic: HIGH = Forward, LOW = Reverse (Active LOW Relay)
  }
  
  void setMotorSpeed(int i, int spd) {
    bool reverse = false;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = true;
    }

    spd = constrain(spd, 0, 255);

    /* Set speed and direction relays */
    if (i == LEFT) { 
      if  (reverse)   { digitalWrite(LEFT_DIRECTION_RELAY_PIN, LOW);  analogWrite(LEFT_MOTOR_PWM_PIN, spd); } // going backward
      else            { digitalWrite(LEFT_DIRECTION_RELAY_PIN, HIGH); analogWrite(LEFT_MOTOR_PWM_PIN, spd); } // going forward
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if  (reverse)   { digitalWrite(RIGHT_DIRECTION_RELAY_PIN, LOW);  analogWrite(RIGHT_MOTOR_PWM_PIN, spd); } // going backward
      else            { digitalWrite(RIGHT_DIRECTION_RELAY_PIN, HIGH); analogWrite(RIGHT_MOTOR_PWM_PIN, spd); } // going forward
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
  
  void setMotorSpeed_PID(int i, int spd) {
    bool reverse = false;
    int pwm_out = 0;
    if (spd < 0)
    {
      spd = -spd;
      reverse = true;
    }
    spd = constrain(spd, 0, 255);
    
    /* Set speed and direction relays */
    if (i == LEFT) { 
      if  (reverse)   { digitalWrite(LEFT_DIRECTION_RELAY_PIN, LOW);  analogWrite(LEFT_MOTOR_PWM_PIN, spd); } // going backward
      else            { digitalWrite(LEFT_DIRECTION_RELAY_PIN, HIGH); analogWrite(LEFT_MOTOR_PWM_PIN, spd); } // going forward
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if  (reverse)   { digitalWrite(RIGHT_DIRECTION_RELAY_PIN, LOW);  analogWrite(RIGHT_MOTOR_PWM_PIN, spd); } // going backward
      else            { digitalWrite(RIGHT_DIRECTION_RELAY_PIN, HIGH); analogWrite(RIGHT_MOTOR_PWM_PIN, spd); } // going forward
    }
  }
  
  void setMotorSpeeds_PID(long leftSpeed, long rightSpeed) {
    setMotorSpeed_PID(LEFT, leftSpeed);
    setMotorSpeed_PID(RIGHT, rightSpeed);
  }

#else
  #error A motor driver must be selected!
#endif

#endif
