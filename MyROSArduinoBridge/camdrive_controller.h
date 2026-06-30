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

#ifndef CAMDRIVE_CONTROLLER_H
#define CAMDRIVE_CONTROLLER_H


typedef struct {
  
  /* PID Parameters */ // To be changed later
  int Kp = 20;
  int Kd = 12;
  int Ki = 0;
  int Ko = 50;
  unsigned char moving = 0; // is the base in motion?
  // PID setpoint info For a Motor      
  int           setPointLocationInFrame;            
  int           PointLocationInFrame;           // InputPointLocation set from ROS2
  long          output;                         // last motor setting / usually PWM value
  
  int   PrevPointLocation;                      // last point location 
  int   PrevInput;                              // last input
  int   ITerm;                                  // integrated term    
  
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
SetPointInfo pointPID;

/*
 * The program should be:
 * 1. Receiving the args sent by the ROS PC
 * 2. Take only the x axis
 * 3. X axis change to deviation or normalized value relative to the middle pixel value
 * 4. Count the error
 * 5. Count the PID value with error fed to the PID block system
 */

void resetPID(){
  
}

void doPID(SetPointInfo *p){
  long Perror;
  long output;
  int   input;

  //input  = p-> - p-> ; // 
  Perror                = p->setPointLocationInFrame - p->PointLocationInFrame;                        // Error    = sp_encoder - encoder
  output                = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevPointLocation  = p->PointLocationInFrame;

  output += p->output;

  // Change this form into 
  
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

void updatePID(){
  // Cek posisi dengan mengecek lokasi point baru
  
//   
//  #endif
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
  
  doPID(&pointPID);   // Compute PID update for each motor
  setMotorSpeeds(leftPID.output, rightPID.output); // Set the motor speeds accordingly
}

#endif

void printDebugMsg(){
  Serial.print("LSet ");
  Serial.print(leftPID.TargetTicksPerFrame);
  Serial.print("\tRSet ");
  Serial.print(rightPID.TargetTicksPerFrame);
  Serial.print("\tLEnc ");
  Serial.print(leftPID.Encoder.ticks);
  Serial.print("\tREnc ");
  Serial.print(rightPID.Encoder.ticks);
  Serial.print("\tLOut ");
  Serial.print(leftPID.output);
  Serial.print("\tROut ");
  Serial.println(rightPID.output);
}
