/*********************************************************************
  Encoder driver definitions on Nano. 
  Encoder reading code on Arduino Nano. This is for production notice, so 
  I won't change or break the code too much when I test some special things.

  The communication schema:
  Encoder (A channel) --> Nano  <--USB--> PC/Jetson with ROS
                              |
                              |
                      "Reset encoder value"

  !!! Important Info !!!
  TODO
  
  Last updated  : September 8, 2025 at 4:35 PM
  TODO          : Go for a Test! ==> OK!

  Have a nice day.~~
 ********************************************************************* */

#include <Wire.h>
#include <util/atomic.h>

// Encoder pins (example: D2 = PD2, D3 = PD3)
#define ENC_PIN_A 2

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

// If you use I2C direction
volatile int8_t dir_sign = +1; // +1 forward, -1 backward

// ========== SETUP ==========
void setup() {
  pinMode(ENC_PIN_A, INPUT_PULLUP);

  // Init previous state
  prev_state = digitalRead(ENC_PIN_A);

  // Enable pin-change interrupt on D2 & D3 (PCINT18, PCINT19 → PCMSK2)
  PCICR |= (1 << PCIE2);       // Enable PCINT[23:16]
  PCMSK2 |= (1 << PCINT18);    // Enable interrupt for D2

  Serial.begin(57600);
  Serial.println("Encoder Nano ready...");
}

// ========== ISR (single-channel edge count) ==========
ISR(PCINT2_vect)
{
  // Read current level
  uint8_t new_state = digitalRead(ENC_PIN_A);
  
  // Count rising edges only (0->1)
  if (prev_state == 0 && new_state == 1) {
    int8_t s = getDirSign();
    if (s != 0) encoder_ticks += s;
    // If s==0 you can either ignore pulses or keep last sign — your choice.
  }
  prev_state = new_state;
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();

    // Check if reset was requested
  if (reset_requested) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoder_ticks = 0;
  }
    reset_requested = false;

    // Serial.println("Encoder ticks have been RESET via I2C command!");
  }

  // if (ticks_requested) {
  //   long ticks_copy;
  //   noInterrupts();
  //   ticks_copy = encoder_ticks;                         // Copy atomically
  //   interrupts();
  //   Wire.write((byte *)&ticks_copy, sizeof(long));      // Send as 4 bytes: [ticks(4)]
  // }

    if (now - last_update >= SAMPLE_TIME) {
    Serial.print("Ticks: ");              // Debug
    Serial.println(encoder_ticks);
  }
  
}

// ========== FAST direction read ==========
static inline int8_t getDirSign()
{
#if USE_DIR_PINS
  // Typical H-bridge truth table:
  // IN1=1, IN2=0 => forward
  // IN1=0, IN2=1 => backward
  // others => brake/coast (treat as 0 or keep last)
  uint8_t in1 = digitalRead(DIR_PIN1);
  uint8_t in2 = digitalRead(DIR_PIN2);

  if (in1 == 1 && in2 == 0) return +1;
  if (in1 == 0 && in2 == 1) return -1;

  return 0; // brake/coast/invalid
#else
  return dir_sign;
#endif
}
