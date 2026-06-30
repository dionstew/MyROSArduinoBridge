/* Functions and type-defs for PID control.
   Taken mostly from Mike Ferguson's ArbotiX code which lives at:   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/*****************************************************************
  Based on diff_controller header file in RosArduinoBridge. 
  Most of the diff_controller codes are still same. I mainly added
  a function prior to my encoder configuration which are added in
  encoder_driver header and compiled program. 
 ***************************************************************** */

#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

/* PID Parameters */
int Kp = 10;
int Kd = 12;
int Ki = 0;
int Ko = 50;
unsigned char moving = 0; // is the base in motion?

#if defined(ENCODER_DISK_20PPR)
    typedef struct {
      // PID setpoint info For a Motor                  
      double        TargetTicksPerFrame;           // target speed in ticks per frame
      EncoderData   Encoder;                       // encoder count
      long          output;                        // last motor setting
      
      long  PrevEnc;                          // last encoder count
      int   PrevInput;                        // last input
      int   ITerm;                            // integrated term    
      
      /* **************************************************************************************************
      int PrevInput;                        // last input
      //int PrevErr;                        // last error
  
      * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
      * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
      */
  
      /* **************************************************************************************************
      int ITerm;                            // integrated term    
      //int Ierror;
  
      * Using integrated term (ITerm) instead of integrated error (Ierror),
      * to allow tuning changes,
      * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
      */
  }
  SetPointInfo;
  SetPointInfo leftPID, rightPID;

  /*
  * Initialize PID variables to zero to prevent startup spikes
  * when turning PID on to start moving
  * In particular, assign both Encoder and PrevEnc the current encoder value
  * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
  * Note that the assumption here is that PID is only turned on
  * when going from stop to moving, that's why we can init everything on zero.
  */
  void resetPID(){
    readEncoder_20PPR(LEFT, leftPID.Encoder);
    readEncoder_20PPR(RIGHT, rightPID.Encoder);

    leftPID.TargetTicksPerFrame   = 0.0;
    leftPID.PrevEnc               = leftPID.Encoder.ticks;
    leftPID.output                = 0;
    leftPID.PrevInput             = 0;
    leftPID.ITerm                 = 0;

    rightPID.TargetTicksPerFrame  = 0.0;
    rightPID.PrevEnc              = rightPID.Encoder.ticks;
    rightPID.output               = 0;
    rightPID.PrevInput            = 0;
    rightPID.ITerm                = 0;
  }

  void doPID(SetPointInfo* p) {
    // PID routine to compute the next motor commands
    long Perror;
    long output;
    int   input;
    
    // Serial.print("TargetTicksperFrame:\t");
    // Serial.println(p->TargetTicksPerFrame);
    //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
    input   = p->Encoder.ticks - p->PrevEnc;                                    // encoder  = currentEncoder - previousEncoder
    Perror  = p->TargetTicksPerFrame - input;                                   // Error    = sp_encoder - encoder

    /*
    * Avoid derivative kick and allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    //  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
    // p->PrevErr = Perror;
    
    output      = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
    //  output = Kp * Perror + p->ITerm + Kd * (input - p->PrevInput);
    //  Serial.println(leftPID.Encoder.ticks-leftPID.PrevEnc);
    p->PrevEnc  = p->Encoder.ticks;

    output += p->output;
    
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM) output = MAX_PWM;
    else if (output <= -MAX_PWM) output = -MAX_PWM;
    /*
    * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    else p->ITerm += Ki * Perror; 

    p->output = output;
    p->PrevInput = input;
  }

  void updatePID() {
    // Read the encoder values and call the PID routine
    readEncoder_20PPR(LEFT, leftPID.Encoder);
    readEncoder_20PPR(RIGHT, rightPID.Encoder);
    
    if (!moving){
      // If we're not moving there is nothing more to do
      /*
      * Reset PIDs once, to prevent startup spikes,
      * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
      * PrevInput is considered a good proxy to detect
      * whether reset has already happened
      */
      if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
      return;
    }
    
    doPID(&rightPID);   // Compute PID update for each motor
    doPID(&leftPID);    // Compute PID update for each motor
    setMotorSpeeds(leftPID.output, rightPID.output); // Set the motor speeds accordingly
  }
  
#elif defined(MEGA_I2C_NANO_ENC_COUNTER)
  typedef struct {
    // PID setpoint info For a Motor                  
    double        TargetTicksPerFrame;           // target speed in ticks per frame
    EncoderData   Encoder;                       // encoder count
    long          output;                        // last motor setting
    
    long  PrevEnc;                          // last encoder count
    int   PrevInput;                        // last input
    int   ITerm;                            // integrated term    
    
    /* **************************************************************************************************
 r   int PrevInput;                        // last input
    //int PrevErr;                        // last error

    * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    */

    /* **************************************************************************************************
    int ITerm;                            // integrated term    
    //int Ierror;

    * Using integrated term (ITerm) instead of integrated error (Ierror),
    * to allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
  }
  SetPointInfo;
  SetPointInfo leftPID, rightPID;

  /*
  * Initialize PID variables to zero to prevent startup spikes
  * when turning PID on to start moving
  * In particular, assign both Encoder and PrevEnc the current encoder value
  * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
  * Note that the assumption here is that PID is only turned on
  * when going from stop to moving, that's why we can init everything on zero.
  */
  void resetPID(){
    readEncoder_I2C(ADDR_NANO_LEFT,   leftPID.Encoder);
    readEncoder_I2C(ADDR_NANO_RIGHT,  rightPID.Encoder);

    leftPID.TargetTicksPerFrame   = 0.0;
    leftPID.PrevEnc               = leftPID.Encoder.ticks;
    leftPID.output                = 0;
    leftPID.PrevInput             = 0;
    leftPID.ITerm                 = 0;

    rightPID.TargetTicksPerFrame  = 0.0;
    rightPID.PrevEnc              = rightPID.Encoder.ticks;
    rightPID.output               = 0;
    rightPID.PrevInput            = 0;
    rightPID.ITerm                = 0;
  }

  void doPID(SetPointInfo* p) {
    // PID routine to compute the next motor commands
    long Perror;
    long output;
    int   input;
    
    // Serial.print("TargetTicksperFrame:\t");
    // Serial.println(p->TargetTicksPerFrame);
    //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
    input   = p->Encoder.ticks - p->PrevEnc;                                    // encoder  = currentEncoder - previousEncoder
    Perror  = p->TargetTicksPerFrame - input;                                   // Error    = sp_encoder - encoder

    /*
    * Avoid derivative kick and allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    //  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
    // p->PrevErr = Perror;
    
    output      = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
    //  output = Kp * Perror + p->ITerm + Kd * (input - p->PrevInput);
    //  Serial.println(leftPID.Encoder.ticks-leftPID.PrevEnc);
    p->PrevEnc  = p->Encoder.ticks;

    output += p->output;
    
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM) output = MAX_PWM;
    else if (output <= -MAX_PWM) output = -MAX_PWM;
    /*
    * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    else p->ITerm += Ki * Perror; 

    p->output = output;
    p->PrevInput = input;
  }

  void updatePID() {
    // Read the encoder values and call the PID routine
    readEncoder_I2C(ADDR_NANO_LEFT, leftPID.Encoder);                  // Read the encoders 
    readEncoder_I2C(ADDR_NANO_RIGHT, rightPID.Encoder);                // Read the encoders 
    
    if (!moving){
      // If we're not moving there is nothing more to do
      /*
      * Reset PIDs once, to prevent startup spikes,
      * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
      * PrevInput is considered a good proxy to detect
      * whether reset has already happened
      */
      if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
      return;
    }
    
    doPID(&rightPID);   // Compute PID update for each motor
    doPID(&leftPID);    // Compute PID update for each motor
    setMotorSpeeds(leftPID.output, rightPID.output); // Set the motor speeds accordingly
  }

// Base controller following the basic setup from ROSArduinoBridge
#else
  typedef struct {
    // PID setpoint info For a Motor                  
    double TargetTicksPerFrame;    // target speed in ticks per frame
    long Encoder;                  // encoder count
    long PrevEnc;                  // last encoder count

    /*
    * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    */
    int PrevInput;                // last input
    //int PrevErr;                   // last error

    /*
    * Using integrated term (ITerm) instead of integrated error (Ierror),
    * to allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    //int Ierror;
    int ITerm;                    //integrated term

    long output;                    // last motor setting
  }
  SetPointInfo;
  SetPointInfo leftPID, rightPID;

  /*
  * Initialize PID variables to zero to prevent startup spikes
  * when turning PID on to start moving
  * In particular, assign both Encoder and PrevEnc the current encoder value
  * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
  * Note that the assumption here is that PID is only turned on
  * when going from stop to moving, that's why we can init everything on zero.
  */
  void resetPID(){
    leftPID.TargetTicksPerFrame = 0.0;
    leftPID.Encoder = readEncoder(LEFT);
    leftPID.PrevEnc = leftPID.Encoder;
    leftPID.output = 0;
    leftPID.PrevInput = 0;
    leftPID.ITerm = 0;

    rightPID.TargetTicksPerFrame = 0.0;
    rightPID.Encoder = readEncoder(RIGHT);
    rightPID.PrevEnc = rightPID.Encoder;
    rightPID.output = 0;
    rightPID.PrevInput = 0;
    rightPID.ITerm = 0;
  }

    /* PID routine to compute the next motor commands */
  void doPID(SetPointInfo * p) {
    long Perror;
    long output;
    int input;

    //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
    input = p->Encoder - p->PrevEnc;                              // encoder  = currentEncoder - previousEncoder
    Perror = p->TargetTicksPerFrame - input;                      // Error    = sp_encoder - encoder

    /*
    * Avoid derivative kick and allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
    // p->PrevErr = Perror;
    output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
    
    p->PrevEnc = p->Encoder;

    output += p->output;
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM)
      output = MAX_PWM;
    else if (output <= -MAX_PWM)
      output = -MAX_PWM;
    else
    /*
    * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
      p->ITerm += Ki * Perror;

    p->output     = output;
    p->PrevInput  = input;
  }

  /* Read the encoder values and call the PID routine */
  void updatePID() {
    /* Read the encoders */
    leftPID.Encoder   = readEncoder(LEFT);
    rightPID.Encoder  = readEncoder(RIGHT);
    
    /* If we're not moving there is nothing more to do */
    if (!moving){
      /*
      * Reset PIDs once, to prevent startup spikes,
      * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
      * PrevInput is considered a good proxy to detect
      * whether reset has already happened
      */
      if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
      return;
    }

    /* Compute PID update for each motor */
    doPID(&rightPID);
    doPID(&leftPID);
    
    /* Set the motor speeds accordingly */
    setMotorSpeeds(leftPID.output, rightPID.output);
  }

#endif
#endif
