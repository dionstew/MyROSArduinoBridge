/*********************************************************************
  Encoder driver definitions on Nano. 
  Encoder reading code on Arduino Nano. This is for production notice, so 
  I won't change or break the code too much when I test some special things.

  The communication schema:
  Encoder (A/B channels) --> Nano <--I2C--> Mega <--USB--> PC/Jetson with ROS
                              |
                              |
                      "Reset encoder value"

  !!! Important Info !!!
  I2C_ADDRESS       : Manually setup address for a corresponding Nano. Make sure
                      you set the same receiving address on the Master.
                      Default: 0x10.
  CMD_RESET_TICKS   : Command for resetting encoder value. Command is written in
                      Hex value 0x01.

  Last updated  : September 8, 2025 at 4:35 PM
  TODO          : Go for a Test! ==> OK!

  Have a nice day.~~
 ********************************************************************* */

#include <Wire.h>
#include <util/atomic.h>

// ========== CONFIG ==========
#define I2C_ADDRESS 0x11   // <-- Change per Nano (e.g. 0x10 for left, 0x11 for right)

// Encoder pins (example: D2 = PD2, D3 = PD3)
#define ENC_PIN_A 2
#define ENC_PIN_B 3

// Command values
#define CMD_RESET_TICKS   0x01    // Command to reset ticks 
#define CMD_REQUEST_TICKS 0x02    // Command to request ticks

// ========== VARIABLES ==========
volatile long encoder_ticks = 0;
volatile uint8_t prev_state = 0;
bool reset_requested = false;
bool ticks_requested = false;

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
  Wire.onRequest(requestEvent);    // Master requests data
  Wire.onReceive(receiveEvent);       // Master sends data

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
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoder_ticks = 0;
    prev_ticks = 0;
  }
    reset_requested = false;
    Serial.println("Encoder ticks have been RESET via I2C command!");
  }

  if (now - last_update >= SAMPLE_TIME) {
    Serial.print("Ticks: ");              // Debug
    Serial.println(encoder_ticks);
  }
}

// ========== I2C CALLBACK ==========
void requestEvent() {
  long ticks_copy;

  // Salin data secara atomik, tapi tidak mematikan I2C interrupt
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ticks_copy = encoder_ticks;
  }

  Wire.write((byte *)&ticks_copy, sizeof(long));
}

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    byte command = Wire.read();

    if (command == CMD_RESET_TICKS) {
      reset_requested = true; // Flag will be processed in loop
    }
    if (command == CMD_REQUEST_TICKS) {
      ticks_requested = true; // Flag will be processed in loop
    }
  }
}