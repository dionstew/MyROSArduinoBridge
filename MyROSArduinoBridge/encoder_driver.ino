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
  // Initializes the Mega Encoder Counter in the 4X Count mode
  MegaEncoderCounter encoders = MegaEncoderCounter(4);  
  
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
  
  void initEncoder(){
          //set as inputs
        DDRD &= ~(1 << LEFT_ENC_PIN_A);
        DDRD &= ~(1 << LEFT_ENC_PIN_B);
        DDRC &= ~(1 << RIGHT_ENC_PIN_A);
        DDRC &= ~(1 << RIGHT_ENC_PIN_B);
  
        //enable pull up resistors
        PORTD |= (1 << LEFT_ENC_PIN_A);
        PORTD |= (1 << LEFT_ENC_PIN_B);
        PORTC |= (1 << RIGHT_ENC_PIN_A);
        PORTC |= (1 << RIGHT_ENC_PIN_B);
  
        // tell pin change mask to listen to left encoder pins
        PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
        // tell pin change mask to listen to right encoder pins
        PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);
  
        // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
        PCICR |= (1 << PCIE1) | (1 << PCIE2);
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
//    Serial.println("Encoder I2C bootup...");
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

#elif defined(ENCODER_DISK_20PPR)
  
  void initEncoderDisk(){
    pinMode(ENC_PIN_A, INPUT_PULLUP);
    pinMode(ENC_PIN_B, INPUT_PULLUP);

    // Init previous state
    prev_stateA = digitalRead(ENC_PIN_A);
    prev_stateB = digitalRead(ENC_PIN_B);
  
    // Enable pin-change interrupt on D2 & D3 (PCINT18, PCINT19 → PCMSK2)
    PCICR |= (1 << PCIE2);       // Enable PCINT[23:16]
    PCMSK2 |= (1 << PCINT18);    // Enable interrupt for D2  
    PCMSK2 |= (1 << PCINT19);    // Enable interrupt for D3
  }

  // ========== ISR ==========
  ISR(PCINT2_vect) {
    // Read both levels (fast): D2=bit2, D3=bit3
    uint8_t pind = PIND;
    uint8_t encA = (pind >> 2) & 0x01;   // D2
    uint8_t encB = (pind >> 3) & 0x01;   // D3
    
    // A: rising edge?
    if (prev_stateA == 0 && encA == 1) {
      int8_t s = getDirSign(DIR_PIN_A1, DIR_PIN_A2);
      if (s != 0) encoder_ticksA += s;
      // If s==0 you can either ignore pulses or keep last sign — your choice.
    }
    // B: rising edge?
    if (prev_stateB == 0 && encB == 1) {
      int8_t s = getDirSign(DIR_PIN_B1, DIR_PIN_B2);
      if (s != 0) encoder_ticksB += s;
    }
    prev_stateA = encA;
    prev_stateB = encB;
  }
  
  void readEncoder_20PPR(int i, EncoderData& data){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      data.ticks = (i == RIGHT) ? encoder_ticksB : encoder_ticksA;
    }
  }
  
  void resetEncoder_20PPR(){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoder_ticksA = 0;
    encoder_ticksB = 0;
//    prev_ticksA = 0;
//    prev_ticksB = 0;
    }
  }

  // ========== FAST direction read ==========
  static inline int8_t getDirSign(int pinA, int pinB)
  {
    int dir_sign;
    #if defined(USE_DIR_PINS)
      // Typical H-bridge truth table:
      // IN1=1, IN2=0 => forward
      // IN1=0, IN2=1 => backward
      // others => brake/coast (treat as 0 or keep last)
      uint8_t in1 = digitalRead(pinA);
      uint8_t in2 = digitalRead(pinB);
    
      if (in1 == 1 && in2 == 0) return +1;
      if (in1 == 0 && in2 == 1) return -1;
    
      return 0; // brake/coast/invalid
    
    #endif
  }
  
#else
#error A encoder driver must be selected!
#endif

#endif
