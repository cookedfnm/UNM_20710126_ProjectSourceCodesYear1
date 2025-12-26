#include <LiquidCrystal.h>
#include <Wire.h>
#include <math.h>

// ===== PINS =====
const int ENA = 3,  IN1 = A3, IN2 = 13;   // MOTOR A (RIGHT)
const int ENB = 11, IN3 = 2,  IN4 = 12;   // MOTOR B (LEFT)
// IR pins still defined but UNUSED now
const int IR_Left = A1, IR_Right = A2;

// LCD: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ===== SPEED / TIME =====
const int NORMAL_SPEED = 135;           // default flat speed
int BASE_SPEED = NORMAL_SPEED;          // current forward speed

// WHEEL TRIM
// Robot curves LEFT → right wheel too fast → RIGHT_WHEEL_OFFSET more negative.
// Robot curves RIGHT → left wheel too fast → LEFT_WHEEL_OFFSET more negative.
const int RIGHT_WHEEL_OFFSET = 20;     // strong slowdown for right wheel
const int LEFT_WHEEL_OFFSET  = -20;     // slight boost for left wheel

// turning / spin speeds (base values)
#define OUTER_TURN       200
#define INNER_TURN       80
#define SPIN_FW          170
#define SPIN_BW          140

// timing parameters
#define TOP_PAUSE_TIME   4000UL         // stop on top of ramp (ms)
#define TURN_360_TIME    3400UL         // spin duration (~360°)
#define RAMP_ENABLE_TIME 2000UL         // ignore ramp until 2 s after start
#define TOP_EXTRA_RUN    52UL           // extra time to drive on top before stopping

// ---- Uphill boost (for going UP the ramp) ----
const int BOOST_SPEED = 255;            // faster speed when starting uphill
const unsigned long BOOST_TIME = 1185UL;// how long boost lasts (ms)
bool rampBoostActive = false;
unsigned long rampBoostStart = 0;

// ---- Slow speed after 360° when going DOWN the ramp ----
const int SLOW_AFTER_SPIN_SPEED = 50;   // very slow speed on descent

// ===== ANGLE / MPU =====
#define RAMP_UP_THRESHOLD  15.0f        // |angle| > 15° = slope
#define FLAT_THRESHOLD      8.0f        // |angle| < 8°  = flat

const uint8_t MPU_ADDR = 0x68;
float angleOffset = 0.0f;

// ===== STEP LOGIC =====
// 0 = flat before ramp
// 1 = going up ramp
// 2 = on top (stop + 4s + 360° spin)
// 3 = going down ramp (slow)
// 4 = after ramp (flat again)
int step = 0;

bool topPauseDone = false, topSpinDone = false;
int slopeCount = 0;
int flatCount  = 0;

// Delay before entering TOP (so we stop nearer the centre)
bool         topDelayActive = false;
unsigned long topDelayStart  = 0;

// time tracking
unsigned long start_time = 0, topPauseStart = 0, topSpinStart = 0;

// ===== MOTOR HELPERS =====
void motorA(int s) {
  if (s > 0) {                      // forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, s);
  } else if (s < 0) {               // reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -s);
  } else {                          // stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorB(int s) {
  if (s > 0) {                      // forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, s);
  } else if (s < 0) {               // reverse
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -s);
  } else {                          // stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// Straight drive with hard-coded trims on each wheel
void moveForward() {
  int rightPWM = BASE_SPEED + RIGHT_WHEEL_OFFSET;   // slow right
  int leftPWM  = BASE_SPEED + LEFT_WHEEL_OFFSET;    // boost left

  // clamp to [0,255] just in case
  if (rightPWM < 0)   rightPWM = 0;
  if (rightPWM > 255) rightPWM = 255;
  if (leftPWM < 0)    leftPWM = 0;
  if (leftPWM > 255)  leftPWM = 255;

  motorA(rightPWM);
  motorB(leftPWM);
}

void spinRight() {
  motorA(-SPIN_BW);
  motorB(SPIN_FW);
}

void spinLeft() {
  motorA(SPIN_FW);
  motorB(-SPIN_BW);
}

void stopMotors() {
  motorA(0);
  motorB(0);
}

// ===== MPU ANGLE =====
float readAngleDegRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                 // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);    // read 6 bytes

  if (Wire.available() < 6) return 0;

  int16_t axRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();         // skip Y
  int16_t azRaw = (Wire.read() << 8) | Wire.read();

  float ax = (float)axRaw;
  float az = (float)azRaw;

  // minus sign so uphill gives positive angle
  float angleRad = atan2(-ax, az);
  return angleRad * 180.0f / PI;
}

float readAngleDeg() {
  return readAngleDegRaw() - angleOffset;  // calibrated so start ≈ 0°
}

// ===== SETUP =====
void setup() {
  // IR pins configured but unused now
  pinMode(IR_Left, INPUT);
  pinMode(IR_Right, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  lcd.begin(16, 2);
  lcd.print("Calibrating...");

  Wire.begin(); // SDA=A4, SCL=A5

  // Wake MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Calibrate angle on flat
  float sum = 0;
  const int N = 60;
  for (int i = 0; i < N; i++) {
    sum += readAngleDegRaw();
    delay(5);
  }
  angleOffset = sum / N;

  lcd.clear();
  moveForward();          // start driving straight
  start_time = millis();
}

// ===== LOOP =====
void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - start_time;

  // read angle
  float angle    = readAngleDeg();
  float absAngle = fabs(angle);

  bool isSlope = (absAngle > RAMP_UP_THRESHOLD);
  bool isFlat  = (absAngle < FLAT_THRESHOLD);

  // simple debouncing for angle decisions
  if (isSlope) slopeCount++; else slopeCount = 0;
  if (isFlat)  flatCount++;  else flatCount  = 0;

  bool rampLogicEnabled = (elapsed >= RAMP_ENABLE_TIME);

  // ===== STEP / RAMP LOGIC =====
  if (rampLogicEnabled) {
    switch (step) {

      case 0: // flat before ramp
        if (slopeCount >= 5) {
          // detected ramp upwards
          step = 1;

          // apply uphill boost
          rampBoostActive = true;
          rampBoostStart = now;
          BASE_SPEED = BOOST_SPEED;
          moveForward();  // update PWM with boosted speed + trims
        }
        break;

      case 1: // going up slope
        if (flatCount >= 5) {
          // reached flat top, run slightly further before stopping
          if (!topDelayActive) {
            topDelayActive = true;
            topDelayStart  = now;
          } else if (now - topDelayStart >= TOP_EXTRA_RUN) {
            // now we are on TOP
            step = 2;
            topPauseDone = false;
            topSpinDone  = false;
            topPauseStart = now;
            stopMotors();
          }
        } else {
          // lost flat, cancel delay
          topDelayActive = false;
        }
        break;

      case 2: // on top (pause + spin handled below)
        // after spin and once we detect slope again → going DOWN
        if (topPauseDone && topSpinDone && slopeCount >= 5) {
          step = 3;
          BASE_SPEED = SLOW_AFTER_SPIN_SPEED;  // slow speed on descent
          moveForward();                       // update PWM with trims
        }
        break;

      case 3: // going down ramp
        if (flatCount >= 5) {
          // reached flat after ramp
          step = 4;
          BASE_SPEED = NORMAL_SPEED;           // back to normal
          moveForward();                       // with trims
        }
        break;

      case 4:
      default:
        // after ramp → just keep going straight with BASE_SPEED
        break;
    }
  }

  // ===== END OF UPHILL BOOST =====
  if (rampBoostActive && (now - rampBoostStart >= BOOST_TIME)) {
    rampBoostActive = false;
    BASE_SPEED = NORMAL_SPEED;
    moveForward();   // apply normal speed again (with trims)
  }

  // ===== TOP PAUSE + SPIN =====
  if (step == 2) {
    if (!topPauseDone) {
      // stay stopped for TOP_PAUSE_TIME
      stopMotors();
      if (now - topPauseStart >= TOP_PAUSE_TIME) {
        topPauseDone = true;
        topSpinStart = now;
        spinRight();    // start 360° spin
      }
    }
    else if (!topSpinDone) {
      // keep spinning until TURN_360_TIME is reached
      spinRight();
      if (now - topSpinStart >= TURN_360_TIME) {
        topSpinDone = true;
        moveForward();  // drive off the top with trims
      }
    }
  }

  // ===== LCD: angle + time + mode =====
  lcd.setCursor(0, 0);
  lcd.print("Ang:");
  lcd.print(angle, 1);
  lcd.print("   ");  // clear leftovers

  unsigned long sec = elapsed / 1000;
  unsigned long ms  = elapsed % 1000;

  lcd.setCursor(0, 1);
  if (sec < 10) lcd.print("0");
  lcd.print(sec);
  lcd.print(":");
  if (ms < 100) lcd.print("0");
  if (ms < 10)  lcd.print("0");
  lcd.print(ms);

  lcd.print(" ");
  const char* label =
    (step == 2 ? "TOP" :
    ((step == 1 || step == 3) ? "ROT" : "LIN"));
  lcd.print(label);
  lcd.print(" ");
}

