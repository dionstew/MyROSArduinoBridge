/* *******************************************************************
  Encoder driver function definitions - by James Nugen
  Added changed - September 3rd, 2025 -  by Dion Setiawan 
      Using transducer encoder-like configuration.
      Encoders -> Nano -->/I2C--> Main Controller (this case is Mega) 
  ******************************************************************** */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H
   
#if defined(ARDUINO_ENC_COUNTER)
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
  
  long readEncoder(int i);
  void resetEncoder(int i);
  void resetEncoders();

#elif defined(MEGA_I2C_NANO_ENC_COUNTER) 
  /******************************************************************************
    THIS IS A FUNNY CONFIGURATION THOUGH, STILL WORTH TO TRY !!!
    EACH NANO COUNTS THE ENCODER AND SENDS THE DATA VIA I2C TO THE MAIN CONTROLLER 
    (MEGA 2560 IN MYCASE OR OTHER CONTROLLER YOU MAY USE)
   ***************************************************************************** */

  #ifndef ADDR_NANO_LEFT
  #define ADDR_NANO_LEFT 0x10
  #endif

  #ifndef ADDR_NANO_RIGHT
  #define ADDR_NANO_RIGHT 0x11
  #endif
  
  #define CMD_REQUEST_TICKS 0x02
  #define CMD_RESET_TICKS   0x01    // Reset command for encoder
  
  struct EncoderData {
    long ticks;
  };

  void initEncoderInterface();
  void readEncoder_I2C(uint8_t address, EncoderData &data);

  void resetEncoder_I2C(int i);
  void resetEncoders_I2C();

#endif

#endif

   
