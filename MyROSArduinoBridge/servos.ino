/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/

#ifdef USE_SERVOS

// Constructor
SweepServo::SweepServo()
{
  this->currentPositionDegrees = 0;
  this->targetPositionDegrees = 0;
  this->lastSweepCommand = 0;
}


// Init
void SweepServo::initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition)
{
  this->servo.attach(servoPin);
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = initPosition;
  this->targetPositionDegrees = initPosition;
  this->servo.write(initPosition);          // <--- penting
  this->lastSweepCommand = millis();
}


// Perform Sweep
void SweepServo::doSweep()
{
  // Get ellapsed time
  unsigned long delta = millis() - this->lastSweepCommand;
  
  // Check if time for a step
  if (delta > this->stepDelayMs) {
    // Check step direction
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;
      this->servo.write(this->currentPositionDegrees);
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;
      this->servo.write(this->currentPositionDegrees);
    }
    // if target == current position, do nothing

    // reset timer
    this->lastSweepCommand = millis();
  }
}

// Go to the next position
void SweepServo::jumpSweep()
{
  this->servo.write(this->targetPositionDegrees);
}

// Set a new target position
void SweepServo::setTargetPosition(int position)
{
  this->targetPositionDegrees = position;
}


// Accessor for servo object
Servo SweepServo::getServo()
{
  return this->servo;
}

#endif
