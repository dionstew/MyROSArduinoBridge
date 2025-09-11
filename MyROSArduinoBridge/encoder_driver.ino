/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   Added on September 3, 2025

   Customized Encoder definitions - by Dion Setiawan
   Custom encoder definitions. Interfacing the controller to Nano which
   runs the speed measurements via I2C. Retrieved Unit are ticks and speed
   (ticks/secs). Ticks are considered as how much the pulse are counted is
   a respective time frame.
   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* Create the encoder shield object */
MegaEncoderCounter encoders = MegaEncoderCounter(4);  // Initializes the Mega Encoder Counter in the 4X Count mode

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return encoders.YAxisGetCount();
  else return encoders.XAxisGetCount();
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) return encoders.YAxisReset();
  else return encoders.XAxisReset();
}


/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}


// ===============================================================//
#elif defined(ARDUINO_ENC_COUNTER)
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int8_t ENC_STATES[] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };  //encoder lookup table

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR(PCINT2_vect) {
  static uint8_t enc_last = 0;

  enc_last <<= 2;                      //shift previous state two places
  enc_last |= (PIND & (3 << 2)) >> 2;  //read the current state into lowest 2 bits

  left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR(PCINT1_vect) {
  static uint8_t enc_last = 0;

  enc_last <<= 2;                      //shift previous state two places
  enc_last |= (PINC & (3 << 4)) >> 4;  //read the current state into lowest 2 bits

  right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return left_enc_pos;
  else return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    left_enc_pos = 0L;
    return;
  } else {
    right_enc_pos = 0L;
    return;
  }
}


/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}


/*******************************************************************

******************************************************************** */

#elif defined(MEGA_I2C_NANO_ENC_COUNTER)
  /***********************************************
    Task: send transfer data request to Nano encoders
    Data: ticks read by the Nano encoders either left
      or right encoder.
    Reset: Reset encoder are done by sending a 0x01 byte
      via I2C to a destined Nano
    ***********************************************/

  #include <Wire.h>

  void initEncoderInterface() {
    Serial.println("Encoder I2C bootup...");
    Wire.begin();
  }

  void readEncoder_I2C(uint8_t address, EncoderData &data) {
    Wire.requestFrom(address, (uint8_t)4);

    if (Wire.available() == 4) {
      // Read ticks (long = 4 bytes, little-endian)
      byte ticks_read[4];
      for (int i = 0; i < 4; i++) {
        ticks_read[i] = Wire.read();
      }
      memcpy(&data.ticks, ticks_read, sizeof(ticks_read));  // Copy bytes into float variable
    }
  }

  /* Wrap the encoder reset function */
  void resetEncoder_I2C(int i) {

    uint8_t targetAddress = (i == LEFT) ? ADDR_NANO_LEFT : ADDR_NANO_RIGHT;

    Wire.beginTransmission(targetAddress);
    Wire.write(CMD_RESET_TICKS);
    Wire.endTransmission();

    // if (i == LEFT){
    //   Wire.beginTransmission(ADDRESS_NANO_AS_LEFT);
    //   Wire.write(ENCODER_RESET_COMMAND); // CMD_RESET_TICKS
    //   Wire.endTransmission();
    //   // left_enc_pos=0L;
    //   return;
    // } else {
    //   Wire.beginTransmission(ADDRESS_NANO_AS_RIGHT);
    //   Wire.write(ENCODER_RESET_COMMAND); // CMD_RESET_TICKS
    //   Wire.endTransmission();
    //   // right_enc_pos=0L;
    //   return;
    // }
  }

  void resetEncoders_I2C() {
    resetEncoder_I2C(LEFT);
    resetEncoder_I2C(RIGHT);
  }

#else
#error A encoder driver must be selected!
#endif

#endif
