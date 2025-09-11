/****************************************************************************
   brakes.h
   Electrical motor brake functions for bicycle electrical driver 
   - by Dion Setiawan
   **************************************************************************/

#ifndef BRAKES_H
#define BRAKES_H

#define ELECTRICAL_BRAKE_RIGHT 28
#define ELECTRICAL_BRAKE_LEFT 30

void initElectricalBrakes();
void setBrake(int i, bool mode);
void setAllBrakes(bool left, bool right);

#endif