#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>

// ================= PINS =================
// Motors
const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = 1;
const int IN4 = 2;

// IR sensors
const int LIR = A1;  // LOW = black
const int MIR = A2;  // HIGH = black (inverted)
const int RIR = A3;  // LOW = black

// Encoders
const int LEFT_ENCODER_PIN  = A4;
const int RIGHT_ENCODER_PIN = A5;

// ================= SPEEDS =================
#define FWD_SPEED      175
#define CORRECT_SPEED  145
#define SWEEP_SPEED    155
// ================= STOP TIMING =================
const unsigned long FINISH_HOLD = 60;  // ms to confirm finish line

// ================= WHEEL CONSTANTS =================
const float CM_PER_PULSE = 0.86;   // your calibrated value

// ================= DISTANCE / TIME =================
volatile unsigned long leftTicks  = 0;
volatile unsigned long rightTicks = 0;

unsigned long startTime = 0;
bool phase1Done = false;

// ================= LCD =================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ================= MOTOR FUNCTIONS =================
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, FWD_SPEED);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, FWD_SPEED);
}

void turnLeftSlow() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, CORRECT_SPEED);

  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENB, CORRECT_SPEED);
}

void turnRightSlow() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENA, CORRECT_SPEED);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, CORRECT_SPEED);
}

void sweepRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENA, SWEEP_SPEED);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, SWEEP_SPEED);
}

void slowCar() {
  // Straight, but slower PWM to reduce momentum
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, 90);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, 90);
}

void stopCar() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ================= ENCODER ISR =================
void leftEncoderISR()  { leftTicks++; }
void rightEncoderISR() { rightTicks++; }

// ================= SETUP =================
void setup() {
  pinMode(LIR, INPUT);
  pinMode(MIR, INPUT);
  pinMode(RIR, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

  lcd.begin(16, 2);
  lcd.print("2-Encoder Bot");
  delay(1000);
  lcd.clear();

  startTime = millis();
}

// ================= LOOP =================
void loop() {

  // ----- Read sensors -----
  bool leftBlack   = (digitalRead(LIR) == LOW);
  bool middleBlack = (digitalRead(MIR) == HIGH);  // inverted
  bool rightBlack  = (digitalRead(RIR) == LOW);

  // ----- Encoder distance -----
  noInterrupts();
  unsigned long lt = leftTicks;
  unsigned long rt = rightTicks;
  interrupts();

  float leftCm  = lt * CM_PER_PULSE;
  float rightCm = rt * CM_PER_PULSE;
  float avgCm   = (leftCm + rightCm) / 2.0;

  float elapsedSec = (millis() - startTime) / 1000.0;

  // ----- LCD distance + time -----
  lcd.setCursor(0, 1);
  lcd.print("D:"); lcd.print(avgCm, 1); lcd.print("cm ");
  lcd.print("T:"); lcd.print(elapsedSec, 1); lcd.print("s ");

  lcd.setCursor(0, 0);

  // ============= PHASE 1: FOLLOW UNTIL ~40 cm ========
  if (!phase1Done) {

    const float PHASE1_TARGET_CM = 40.0;
    const float PHASE1_MARGIN_CM = 2.0;   // stop ~2 cm early to account for momentum

    // 1) Predictive stop slightly before 40 cm
    if (avgCm >= PHASE1_TARGET_CM - PHASE1_MARGIN_CM) {
      stopCar();
      lcd.print("40cm Reached  ");
      delay(3000);          // pause 3 seconds
      phase1Done = true;
      return;
    }

    // 2) Slow zone: between 20 cm and ~38 cm
    if (avgCm >= 20.0) {
      slowCar();
      lcd.print("SLOW ZONE P1  ");
      return;   // override turning in slow zone
    }

    // 3) Normal Phase 1 line following (0–20 cm)
    if (middleBlack) {
      moveForward();
      lcd.print("RUNNING P1    ");
      return;
    }
    if (leftBlack) {
      lcd.print("COR R P1      ");
      while (!digitalRead(MIR)) turnRightSlow();
      return;
    }
    if (rightBlack) {
      lcd.print("COR L P1      ");
      while (!digitalRead(MIR)) turnLeftSlow();
      return;
    }
    sweepRight();
    lcd.print("SWEEP P1      ");
    return;
  }
  // ============= PHASE 2: NORMAL LOGIC ===============

  // ----- Majority-based finish line detection -----
  static unsigned long blackStart = 0;
  unsigned long now = millis();

  int blackCount = 0;
  if (leftBlack)   blackCount++;
  if (middleBlack) blackCount++;
  if (rightBlack)  blackCount++;

  // Consider finish line if 2 or more sensors are black
  if (blackCount >= 2) {
    if (blackStart == 0) blackStart = now;  // start timer
  } else {
    blackStart = 0;                          // reset if not enough blacks
  }

  // Hold condition: if ≥2 sensors black for FINISH_HOLD ms → finish
  if (blackStart != 0 && (now - blackStart >= FINISH_HOLD)) {
    stopCar();
    lcd.print("FINISH LINE   ");
    lcd.setCursor(0, 1);
    lcd.print("D:"); lcd.print(avgCm, 1); lcd.print("cm ");
    lcd.print("T:"); lcd.print(elapsedSec, 1); lcd.print("s ");
    while (true);  // stop forever
  }

  // ----- Normal line following -----
  if (middleBlack) {
    moveForward();
    lcd.print("RUNNING       ");
    return;
  }

  if (leftBlack) {
    lcd.print("COR R         ");
    while (!digitalRead(MIR)) turnRightSlow();
    return;
  }

  if (rightBlack) {
    lcd.print("COR L         ");
    while (!digitalRead(MIR)) turnLeftSlow();
    return;
    
  }

  sweepRight();
  lcd.print("SWEEP R       ");
}
