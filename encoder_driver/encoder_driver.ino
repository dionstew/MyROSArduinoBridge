/*********************************************************************
  Encoder driver definitions on Nano.
  This code is for demonstration and brainstorming algos only.
  Make sure to mark your added code, comment the unwanted code so you
  won't miss any change or older version. This might make the code doesn't
  look clean. You can decide your own way tho. :)

  Have a nice day.~~
 ********************************************************************* */

#include <Wire.h>

// ========== CONFIG ==========
#define I2C_ADDRESS 0x10   // <-- Change per Nano (e.g. 0x10 for left, 0x11 for right)

// Encoder pins (example: D2 = PD2, D3 = PD3)
#define ENC_PIN_A 2
#define ENC_PIN_B 3

// Command codes
#define CMD_RESET_TICKS 0x01   // Command to reset encoder ticks

// ========== VARIABLES ==========
volatile long encoder_ticks = 0;
volatile uint8_t prev_state = 0;
bool reset_requested = false;

// Lookup table for quadrature decoding
// Index = (old_state << 2) | new_state
const int8_t ENC_STATES[] = {0, -1, +1, 0,
                             +1, 0, 0, -1,
                             -1, 0, 0, +1,
                             0, +1, -1, 0};

// For velocity estimation
long prev_ticks = 0;
float velocity_tick = 0.0;
unsigned long last_update = 0;
const unsigned long SAMPLE_TIME = 100; // ms
long current_ticks;

// ========== SETUP ==========
void setup() {
  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);

  // Init previous state
  prev_state = (digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B);

  // Enable pin-change interrupt on D2 & D3 (PCINT18, PCINT19 → PCMSK2)
  PCICR |= (1 << PCIE2);       // Enable PCINT[23:16]
  PCMSK2 |= (1 << PCINT18);    // Enable interrupt for D2
  PCMSK2 |= (1 << PCINT19);    // Enable interrupt for D3

  // I2C Slave setup
  Wire.begin(I2C_ADDRESS);
  // Wire.setClock(400000L);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);   // Master sends commands

  Serial.begin(9600);
  Serial.println("Encoder Nano ready...");
}

// ========== ISR ==========
ISR(PCINT2_vect) {
  uint8_t new_state = (digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B);
  uint8_t index = (prev_state << 2) | new_state;
  encoder_ticks += ENC_STATES[index];
  prev_state = new_state;
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();

  // Check if reset was requested
  if (reset_requested) {
    noInterrupts();
    encoder_ticks = 0;
    prev_ticks = 0;
    interrupts();
    reset_requested = false;

    Serial.println("Encoder ticks have been RESET via I2C command!");
  }

  if (now - last_update >= SAMPLE_TIME) {
    
    //     // ==== SIMULASI ENCODER ====
    // encoder_ticks += 5;  // misalnya encoder bergerak +5 ticks setiap 100ms
    // if (encoder_ticks > 10000) {
    //   encoder_ticks = 0; // reset jika sudah lewat 1000 ticks
    // }
    current_ticks = encoder_ticks;
    
    velocity_tick = (float)(current_ticks - prev_ticks) / (SAMPLE_TIME / 1000.0); // ticks/sec
    prev_ticks = current_ticks;
    last_update = now;

    // Debug
    Serial.print("Ticks: ");
    Serial.print(current_ticks);
    Serial.print(" | Vel: ");
    Serial.println(velocity_tick);
  }
}

// ========== I2C CALLBACK ==========
void requestEvent() {
  // long ticks_copy;
  // float vel_tick_copy;

  // // Copy atomically
  // noInterrupts();
  // ticks_copy = current_ticks;
  // vel_tick_copy = velocity_tick;
  // interrupts();

  // Send as 8 bytes: [ticks(4), velocity(4)]
  Wire.write((byte *)&encoder_ticks, sizeof(current_ticks));
  // Wire.write((byte *)&vel_tick_copy, sizeof(float));
}

// Receive commands from Mega
void receiveEvent(int numBytes) {
  while (Wire.available()) {
    byte command = Wire.read();

    if (command == CMD_RESET_TICKS) {
      reset_requested = true; // Flag will be processed in loop
    }
  }
}
