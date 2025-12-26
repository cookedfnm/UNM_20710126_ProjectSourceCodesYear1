#include <LiquidCrystal.h>

// ================== PIN DEFINITIONS ==================
const int ENA = 3;    // Right motor PWM
const int IN1 = A3;   // Right motor direction 1
const int IN2 = 13;   // Right motor direction 2

const int ENB = 11;   // Left motor PWM
const int IN3 = 2;    // Left motor direction 1
const int IN4 = 12;   // Left motor direction 2

// Ultrasonic sensor pins
const int TRIG_PIN = A2;  // Trigger
const int ECHO_PIN = A1;  // Echo

// LCD: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ================== SPEED SETTINGS ===================
// Forward speed (constant)
const int FORWARD_SPEED = 120;

// Turning / avoiding speed 
int TURN_SPEED = 160;

// How close before we react (cm)
const int OBSTACLE_DISTANCE_CM = 20;

// How long to turn right when avoiding (ms)
const unsigned long TURN_TIME_MS = 500;

// ================== MOTOR HELPERS ====================
void motorA(int s) {
  if (s > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, s);
  } else if (s < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -s);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorB(int s) {
  if (s > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, s);
  } else if (s < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -s);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// Go straight forward at constant speed
void moveForward() {
  motorA(FORWARD_SPEED);   // right wheel
  motorB(FORWARD_SPEED);   // left wheel
}

// Spin / turn right at TURN_SPEED
void turnRight() {
  // Right wheel backward, left wheel forward
  motorA(-TURN_SPEED);
  motorB(TURN_SPEED);
}

// Stop both motors
void stopMotors() {
  motorA(0);
  motorB(0);
}

// ================== ULTRASONIC HELPER =================
float readDistanceCM() {
  // Trigger a 10µs pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time (timeout ~20ms)
  long duration = pulseIn(ECHO_PIN, HIGH, 20000);

  if (duration == 0) {
    // No echo detected within timeout – treat as "no obstacle"
    return 999.0;
  }

  // Sound speed ≈ 0.0343 cm/µs, divide by 2 for round trip
  float distance = duration * 0.0343 / 2.0;
  return distance;
}

// ================== SETUP ============================
void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LCD init
  lcd.begin(16, 2);
  lcd.print("Ultrasonic Car");
  delay(1000);
  lcd.clear();
}

// ================== MAIN LOOP ========================
void loop() {
  float distance = readDistanceCM();

  // Show distance on LCD
  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print(distance, 1);
  lcd.print("cm   "); // spaces to clear old chars

  // Simple behaviour:
  // If obstacle is closer than threshold, stop, wait, then turn right
  if (distance < OBSTACLE_DISTANCE_CM) {
    lcd.setCursor(0, 1);
    lcd.print("Avoiding...    ");

    stopMotors();
    delay(1000);             // stop for 1 second

    turnRight();
    delay(TURN_TIME_MS);     // turn right for a bit

    stopMotors();
    delay(100);              // small pause
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Forward        ");
    moveForward();
  }

  // Small delay to avoid spamming ultrasonic sensor too fast
  delay(50);
}

