#include <LiquidCrystal.h>
#include <Wire.h>
#include <math.h>

// ===== PINS =====
const int ENA = 3,  IN1 = A3, IN2 = 13;   // MOTOR A (RIGHT)
const int ENB = 11, IN3 = 2,  IN4 = 12;   // MOTOR B (LEFT)

// LCD: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ===== SPEED / TIME =====
const int NORMAL_SPEED = 135;            // default flat speed
int BASE_SPEED = NORMAL_SPEED;           // current forward speed

// WHEEL TRIM
const int RIGHT_WHEEL_OFFSET = 20;       // Slow down right
const int LEFT_WHEEL_OFFSET  = -20;      // Boost left

// Turning / spin speeds
#define SPIN_FW          170
#define SPIN_BW          140

// Timing parameters
#define TOP_PAUSE_TIME   4000UL          // stop on top (ms)
#define TURN_360_TIME    3400UL          // spin duration (~360°)
#define RAMP_ENABLE_TIME 2000UL          // ignore ramp until 2s
#define TOP_EXTRA_RUN    52UL            // extra run on top

// ---- Uphill boost ----
const int BOOST_SPEED = 255;             
const unsigned long BOOST_TIME = 1185UL; 
bool rampBoostActive = false;
unsigned long rampBoostStart = 0;

// ---- Slow Descent ----
const int SLOW_AFTER_SPIN_SPEED = 50;    

// ===== ANGLE / MPU =====
#define RAMP_UP_THRESHOLD  15.0f         
#define FLAT_THRESHOLD     8.0f          

const uint8_t MPU_ADDR = 0x68;
float angleOffset = 0.0f;

// ===== NEW VARIABLE FOR MEASUREMENT =====
float maxRampAngle = 0.0;  // Stores the biggest angle seen

// ===== STEP LOGIC =====
// 0=Flat, 1=Up, 2=Top, 3=Down, 4=Done
int step = 0;

bool topPauseDone = false, topSpinDone = false;
int slopeCount = 0;
int flatCount  = 0;

bool topDelayActive = false;
unsigned long topDelayStart = 0;

unsigned long start_time = 0, topPauseStart = 0, topSpinStart = 0;

// ===== MOTOR HELPERS =====
void motorA(int s) {
  if (s > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, s); }
  else if (s < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, -s); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0); }
}

void motorB(int s) {
  if (s > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, s); }
  else if (s < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, -s); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0); }
}

void moveForward() {
  int rightPWM = BASE_SPEED + RIGHT_WHEEL_OFFSET;
  int leftPWM  = BASE_SPEED + LEFT_WHEEL_OFFSET;
  
  if (rightPWM < 0) rightPWM = 0; if (rightPWM > 255) rightPWM = 255;
  if (leftPWM < 0)  leftPWM = 0;  if (leftPWM > 255)  leftPWM = 255;

  motorA(rightPWM);
  motorB(leftPWM);
}

void spinRight() { motorA(-SPIN_BW); motorB(SPIN_FW); }
void stopMotors() { motorA(0); motorB(0); }

// ===== MPU ANGLE =====
float readAngleDegRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6); 

  if (Wire.available() < 6) return 0;
  int16_t axRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip Y
  int16_t azRaw = (Wire.read() << 8) | Wire.read();

  float angleRad = atan2(-(float)axRaw, (float)azRaw);
  return angleRad * 180.0f / PI;
}

float readAngleDeg() {
  return readAngleDegRaw() - angleOffset;
}

// ===== SETUP =====
void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  lcd.begin(16, 2);
  lcd.print("Calibrating...");
  Wire.begin(); 
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  float sum = 0;
  for (int i = 0; i < 60; i++) { sum += readAngleDegRaw(); delay(5); }
  angleOffset = sum / 60.0;

  lcd.clear();
  moveForward();
  start_time = millis();
}

// ===== LOOP =====
void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - start_time;

  float angle    = readAngleDeg();
  float absAngle = fabs(angle);

  // Debouncing logic
  if (absAngle > RAMP_UP_THRESHOLD) slopeCount++; else slopeCount = 0;
  if (absAngle < FLAT_THRESHOLD)    flatCount++;  else flatCount  = 0;

  bool rampLogicEnabled = (elapsed >= RAMP_ENABLE_TIME);

  if (rampLogicEnabled) {
    switch (step) {

      case 0: // Flat
        if (slopeCount >= 5) {
          step = 1;
          rampBoostActive = true;
          rampBoostStart = now;
          BASE_SPEED = BOOST_SPEED;
          moveForward();
        }
        break;

      case 1: // Going UP (MEASURE HERE!)
        // --- NEW MEASUREMENT LOGIC ---
        // If current angle is bigger than what we have seen, save it.
        if (absAngle > maxRampAngle) {
          maxRampAngle = absAngle;
        }
        // -----------------------------

        if (flatCount >= 5) {
          if (!topDelayActive) {
            topDelayActive = true;
            topDelayStart  = now;
          } else if (now - topDelayStart >= TOP_EXTRA_RUN) {
            step = 2;
            topPauseDone = false; topSpinDone = false;
            topPauseStart = now;
            stopMotors();
          }
        } else {
          topDelayActive = false;
        }
        break;

      case 2: // Top Logic (Stop + Spin)
        if (topPauseDone && topSpinDone && slopeCount >= 5) {
          step = 3;
          BASE_SPEED = SLOW_AFTER_SPIN_SPEED; 
          moveForward();
        }
        break;

      case 3: // Going Down
        if (flatCount >= 5) {
          step = 4;
          BASE_SPEED = NORMAL_SPEED; 
          moveForward();
          lcd.clear(); // Clear screen once for the final result
        }
        break;

      case 4: // Finished
        // Drive forward forever, but show RESULT on LCD
        moveForward();
        
        // --- DISPLAY RESULT ---
        lcd.setCursor(0, 0);
        lcd.print("Ramp Angle:");
        lcd.setCursor(0, 1);
        lcd.print(maxRampAngle, 1); // Show the MAX angle we measured
        lcd.print((char)223);       // Degree symbol
        lcd.print(" Measured");
        return; // Skip the standard LCD update below
        break;
    }
  }

  // Handle Boost Timer
  if (rampBoostActive && (now - rampBoostStart >= BOOST_TIME)) {
    rampBoostActive = false;
    BASE_SPEED = NORMAL_SPEED;
    moveForward();
  }

  // Handle Top Events
  if (step == 2) {
    if (!topPauseDone) {
      stopMotors();
      if (now - topPauseStart >= TOP_PAUSE_TIME) {
        topPauseDone = true;
        topSpinStart = now;
        spinRight(); 
      }
    }
    else if (!topSpinDone) {
      spinRight();
      if (now - topSpinStart >= TURN_360_TIME) {
        topSpinDone = true;
        moveForward(); 
      }
    }
  }

  // ===== STANDARD LCD (Only runs for steps 0, 1, 2, 3) =====
  lcd.setCursor(0, 0);
  lcd.print("Ang:"); lcd.print(angle, 1); lcd.print("   ");

  unsigned long sec = elapsed / 1000;
  lcd.setCursor(0, 1);
  if (sec < 10) lcd.print("0"); lcd.print(sec);
  lcd.print("s ");
  const char* label = (step == 2 ? "TOP" : ((step == 1 || step == 3) ? "ROT" : "LIN"));
  lcd.print(label);
}
