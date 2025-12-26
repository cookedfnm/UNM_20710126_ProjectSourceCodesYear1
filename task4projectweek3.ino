#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
// ================= PIN CONFIGURATION =================

// Right motor
const int ENA = 3;
const int IN1 = A3;
const int IN2 = 13;

// Left motor
const int ENB = 11;
const int IN3 = 2;
const int IN4 = 12;

// LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ================= BLUETOOTH =================
SoftwareSerial BT(1, A2);   // RX, TX  (HC-05 TX->D1, RX->A2)

// ================= SPEED SETTINGS =================
int BASE_SPEED = 170;
int TURN_SPEED = 200;

const int RIGHT_WHEEL_OFFSET = 20;
const int LEFT_WHEEL_OFFSET  = -20;

// ================= TIMEOUT =================
const unsigned long CMD_TIMEOUT = 1000;
unsigned long lastCmdTime = 0;

// ================= MODES =================
enum MotionMode {
  MODE_STOP,
  MODE_FWD,
  MODE_BACK,
  MODE_LEFT,
  MODE_RIGHT,
  MODE_FWD_LEFT,
  MODE_FWD_RIGHT
};

MotionMode currentMode = MODE_STOP;
MotionMode lastShownMode = MODE_STOP;

// ================= MOTOR HELPERS =================

void motorA(int s) {
  if (s > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, s); }
  else if (s < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, -s); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0); }
}

void motorB(int s) {
  if (s > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, s); }
  else if (s < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, -s); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0); }
}

void stopMotors() {
  motorA(0);
  motorB(0);
}

// Straight forward
void forwardStraight() {
  int r = constrain(BASE_SPEED + RIGHT_WHEEL_OFFSET, 0, 255);
  int l = constrain(BASE_SPEED + LEFT_WHEEL_OFFSET, 0, 255);
  motorA(r);
  motorB(l);
}

// Straight backward
void backwardStraight() {
  motorA(-BASE_SPEED);
  motorB(-BASE_SPEED);
}

// ================= SHARPER STRAFES  =================

// Forward + LEFT 
void forwardRightArc() {
  int r = (BASE_SPEED * 0.2) + RIGHT_WHEEL_OFFSET;  // inside wheel (SLOW)
  int l = BASE_SPEED + LEFT_WHEEL_OFFSET;           // outside wheel (FAST)
  r = constrain(r, 0, 255);
  l = constrain(l, 0, 255);
  motorA(r);
  motorB(l);
}

// Forward + RIGHT 
void forwardLeftArc() {
  int r = BASE_SPEED + RIGHT_WHEEL_OFFSET;          // outside wheel FAST
  int l = (BASE_SPEED * 0.2) + LEFT_WHEEL_OFFSET;   // inside wheel SLOW
  r = constrain(r, 0, 255);
  l = constrain(l, 0, 255);
  motorA(r);
  motorB(l);
}

// Spins
void spinLeft()  { motorA(-TURN_SPEED); motorB(TURN_SPEED); }
void spinRight() { motorA(TURN_SPEED);  motorB(-TURN_SPEED); }

// ================= LCD =================

void updateLCD(MotionMode mode) {
  if (mode == lastShownMode) return;
  lastShownMode = mode;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BT Arrow Car");
  lcd.setCursor(0, 1);

  switch (mode) {
    case MODE_STOP:       lcd.print("STOP"); break;
    case MODE_FWD:        lcd.print("FORWARD"); break;
    case MODE_BACK:       lcd.print("BACKWARD"); break;
    case MODE_LEFT:       lcd.print("LEFT SPIN"); break;
    case MODE_RIGHT:      lcd.print("RIGHT SPIN"); break;
    case MODE_FWD_LEFT:   lcd.print("FWD + LEFT"); break;
    case MODE_FWD_RIGHT:  lcd.print("FWD + RIGHT"); break;
  }
}

// ================= COMMAND PROCESSING =================

void handleCommand(char cmd, unsigned long now) {
  lastCmdTime = now;

  switch (cmd) {
    // Forward
    case 'U': case 'u':
    case 'F': case 'f':
    case '2':
      currentMode = MODE_FWD;
      break;

    // Backward
    case 'D': case 'd':
    case 'B': case 'b':
    case '8':
      currentMode = MODE_BACK;
      break;

    // Turns
    case 'L': case 'l':
    case '4':
      currentMode = MODE_LEFT;
      break;

    case 'R': case 'r':
    case '6':
      currentMode = MODE_RIGHT;
      break;

    // ================= NEW COMBOS =================
    // F + L → app sends G
    case 'G': case 'g':
      currentMode = MODE_FWD_RIGHT;
      break;

    // F + R → app sends H
    case 'H': case 'h':
      currentMode = MODE_FWD_LEFT;
      break;

    // Stop
    case 'S': case 's': case '0':
      currentMode = MODE_STOP;
      break;

    default:
      break;
  }
}

// ================= APPLY MOTION =================

void applyMotion() {
  switch (currentMode) {
    case MODE_STOP: stopMotors(); break;
    case MODE_FWD: forwardStraight(); break;
    case MODE_BACK: backwardStraight(); break;
    case MODE_LEFT: spinLeft(); break;
    case MODE_RIGHT: spinRight(); break;

    // Inverted strafes
    case MODE_FWD_LEFT: forwardLeftArc(); break;
    case MODE_FWD_RIGHT: forwardRightArc(); break;
  }

  updateLCD(currentMode);
}

// ================= SETUP =================

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  lcd.begin(16, 2);
  lastShownMode = (MotionMode)(-1);
  updateLCD(MODE_STOP);

  BT.begin(9600);
  stopMotors();
  lastCmdTime = millis();
}

// ================= LOOP =================

void loop() {
  unsigned long now = millis();

  while (BT.available()) {
    char cmd = BT.read();
    if (cmd == '\n' || cmd == '\r') continue;
    handleCommand(cmd, now);
  }

  if (now - lastCmdTime > CMD_TIMEOUT)
    currentMode = MODE_STOP;

  applyMotion();
}
