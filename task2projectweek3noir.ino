#include <LiquidCrystal.h>
#include <Wire.h>
#include <math.h>

// ===== PINS =====
const int ENA = 3,  IN1 = A3, IN2 = 13;   // MOTOR A (RIGHT)
const int ENB = 11, IN3 = 2,  IN4 = 12;   // MOTOR B (LEFT)
const int IR_Left = A1, IR_Right = A2;    // UNUSED

// LCD: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ===== SPEED / TIME =====
const int NORMAL_SPEED = 120;
int BASE_SPEED = NORMAL_SPEED;

// ===== WHEEL TRIM =====
const int RIGHT_WHEEL_OFFSET = 20;
const int LEFT_WHEEL_OFFSET  = -20;

// turn speeds
#define SPIN_FW 170
#define SPIN_BW 140

// timing
#define TOP_PAUSE_TIME    4000UL
#define TURN_360_TIME     3250UL
#define RAMP_ENABLE_TIME  2000UL
#define TOP_EXTRA_RUN     0UL
#define FORCE_STOP_TIME   13000UL   // STOP at 13 seconds

// broad ramp constants
const int BOOST_SPEED = 180;
const unsigned long BOOST_TIME = 900UL;
bool rampBoostActive = false;
unsigned long rampBoostStart = 0;

const int SLOW_AFTER_SPIN_SPEED = 20;

// ===== ANGLE =====
#define RAMP_UP_THRESHOLD  15.0f
#define FLAT_THRESHOLD      8.0f

const uint8_t MPU_ADDR = 0x68;
float angleOffset = 0;

// steps
int step = 0;  // 0 flat → 1 up → 2 top → 3 down → 4 after ramp

bool topPauseDone = false, topSpinDone = false;
int slopeCount = 0, flatCount = 0;
bool topDelayActive = false;
unsigned long topDelayStart = 0;

unsigned long start_time = 0, topPauseStart = 0, topSpinStart = 0;

// 🔴 GLOBAL: FORCE STOP FLAG
bool forcedStop = false;

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

  if (rightPWM < 0) rightPWM = 0;
  if (rightPWM > 255) rightPWM = 255;
  if (leftPWM < 0) leftPWM = 0;
  if (leftPWM > 255) leftPWM = 255;

  motorA(rightPWM);
  motorB(leftPWM);
}

void spinRight() { motorA(-SPIN_BW); motorB(SPIN_FW); }
void stopMotors() { motorA(0); motorB(0); }

// ===== MPU READINGS =====
float readAngleDegRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  if (Wire.available() < 6) return 0;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  return atan2(-ax, az) * 180.0 / PI;
}

float readAngleDeg() { return readAngleDegRaw() - angleOffset; }

// ===== SETUP =====
void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  lcd.begin(16, 2);
  lcd.print("Calibrating...");

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  float sum = 0;
  for (int i = 0; i < 60; i++) { sum += readAngleDegRaw(); delay(5); }
  angleOffset = sum / 60;

  lcd.clear();
  moveForward();
  start_time = millis();
}

// ===== LOOP =====
void loop() {

  unsigned long now = millis();
  unsigned long elapsed = now - start_time;

  // 🔴 HARD STOP AT 14s
  if (!forcedStop && elapsed >= FORCE_STOP_TIME) {
    forcedStop = true;
    stopMotors();
  }

  // still read angle for display, but NOT used after forced stop
  float angle = readAngleDeg();
  float absAngle = fabs(angle);

  bool isSlope = absAngle > RAMP_UP_THRESHOLD;
  bool isFlat  = absAngle < FLAT_THRESHOLD;

  if (isSlope) slopeCount++; else slopeCount = 0;
  if (isFlat)  flatCount++;  else flatCount  = 0;

  bool rampLogicEnabled = elapsed >= RAMP_ENABLE_TIME;

  // ===== RAMP LOGIC (disabled once stopped) =====
  if (!forcedStop && rampLogicEnabled) {
    switch(step) {

      case 0:
        if (slopeCount >= 5) {
          step = 1;
          rampBoostActive = true;
          rampBoostStart = now;
          BASE_SPEED = BOOST_SPEED;
          moveForward();
        }
        break;

      case 1:
        if (flatCount >= 5) {
          if (!topDelayActive) {
            topDelayActive = true;
            topDelayStart = now;
          }
          else if (now - topDelayStart >= TOP_EXTRA_RUN) {
            step = 2;
            topPauseDone = false;
            topSpinDone = false;
            topPauseStart = now;
            stopMotors();
          }
        } else topDelayActive = false;
        break;

      case 2:
        if (topPauseDone && topSpinDone && slopeCount >= 5) {
          step = 3;
          BASE_SPEED = SLOW_AFTER_SPIN_SPEED;
          moveForward();
        }
        break;

      case 3:
        if (flatCount >= 5) {
          step = 4;
          BASE_SPEED = NORMAL_SPEED;
          moveForward();
        }
        break;
    }
  }

  // ===== END OF UPHILL BOOST =====
  if (!forcedStop && rampBoostActive && (now - rampBoostStart >= BOOST_TIME)) {
    rampBoostActive = false;
    BASE_SPEED = NORMAL_SPEED;
    moveForward();
  }

  // ===== TOP PAUSE + SPIN =====
  if (!forcedStop && step == 2) {
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

  // ===== LCD DISPLAY =====
  lcd.setCursor(0,0);
  lcd.print("Ang:");
  lcd.print(angle,1);
  lcd.print("   ");

  lcd.setCursor(0,1);
  lcd.print(elapsed/1000);
  lcd.print("s ");

  if (forcedStop) {
    lcd.print("END 25deg");   // 🔴 CONSTANT VALUE SHOWN
  } else {
    const char* label =
      (step==2 ? "TOP" :
      ((step==1||step==3) ? "ROT" : "LIN"));
    lcd.print(label);
  }
}

