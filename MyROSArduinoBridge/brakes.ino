/****************************************************************************
   brakes.ino
   Electrical motor brakes for bicycle electrical driver - by Dion Setiawan
  *************************************************************************** */

#ifdef USE_BASE

  void initElectricalBrakes() {
    // setup pin Brake as OUTPUT
    pinMode(ELECTRICAL_BRAKE_LEFT,  OUTPUT);
    pinMode(ELECTRICAL_BRAKE_RIGHT, OUTPUT);

    // setup brake initialization, aktifkan brake untuk awal
    digitalWrite(ELECTRICAL_BRAKE_LEFT,   LOW);
    digitalWrite(ELECTRICAL_BRAKE_RIGHT,  LOW);
  }

  void setBrake(int i, bool is_On) {
    // change the condition of brakes
    if (i==LEFT){
      if (is_On)  {digitalWrite(ELECTRICAL_BRAKE_LEFT, LOW);   }
      else        {digitalWrite(ELECTRICAL_BRAKE_LEFT, HIGH);  }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if (is_On)  {digitalWrite(ELECTRICAL_BRAKE_RIGHT, LOW);   }
      else        {digitalWrite(ELECTRICAL_BRAKE_RIGHT, HIGH);  }  
    }
  }

  void setAllBrakes(bool leftBrakeStatus, bool rightBrakeStatus) {
    // receive Condition for brakes, used in the main code
    setBrake(LEFT, leftBrakeStatus);
    setBrake(RIGHT, rightBrakeStatus);
  }

#else
#error Brake must be initialized!
#endif