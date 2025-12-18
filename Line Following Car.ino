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
const int LEFT_ENCODER_PIN  = A4;  // left wheel D0
const int RIGHT_ENCODER_PIN = A5;  // right wheel D0

// ================= SPEEDS =================
#define FWD_SPEED      175
#define CORRECT_SPEED  145
#define SWEEP_SPEED    155

// ================= STOP TIMING =================
const unsigned long FINISH_HOLD = 60;   // ms needed to confirm finish black bar

// ================= WHEEL CONSTANTS =================
// Wheel diameter ~6.5 cm → circumference ≈ 20.42 cm
const float WHEEL_DIAMETER      = 6.5;
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;  
const int   PULSES_PER_REV      = 20;

// Calibration factor correction (found from your testing)
const float CM_PER_PULSE = 0.86; 

// ================= DISTANCE / TIME =================
volatile unsigned long leftTicks  = 0;
volatile unsigned long rightTicks = 0;

unsigned long startTime = 0;

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

void stopCar() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ================= ENCODER ISR =================
void leftEncoderISR() {
  leftTicks++;
}

void rightEncoderISR() {
  rightTicks++;
}

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

  // Encoder pins
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachPinChangeInterrupt(
    digitalPinToPinChangeInterrupt(LEFT_ENCODER_PIN), 
    leftEncoderISR, 
    RISING
  );
  attachPinChangeInterrupt(
    digitalPinToPinChangeInterrupt(RIGHT_ENCODER_PIN),
    rightEncoderISR,
    RISING
  );

  // LCD
  lcd.begin(16, 2);
  lcd.print("2-Encoder Bot");
  delay(1000);
  lcd.clear();

  startTime = millis();
}

// ================= LOOP =================
void loop() {
  // -------- READ SENSORS ----------
  bool leftBlack   = (digitalRead(LIR) == LOW);
  bool middleBlack = (digitalRead(MIR) == HIGH);  // inverted
  bool rightBlack  = (digitalRead(RIR) == LOW);

  // -------- COPY TICKS SAFELY ----------
  noInterrupts();
  unsigned long lt = leftTicks;
  unsigned long rt = rightTicks;
  interrupts();

  // -------- COMPUTE DISTANCE ----------
  float leftCm  = lt * CM_PER_PULSE;
  float rightCm = rt * CM_PER_PULSE;
  float avgCm   = (leftCm + rightCm) / 2.0;

  // -------- COMPUTE TIME ----------
  float elapsedSec = (millis() - startTime) / 1000.0;

  // -------- LCD LINE 2 (DIST + TIME) ----------
  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(avgCm, 1);
  lcd.print("cm ");

  lcd.print("T:");
  lcd.print(elapsedSec, 1);
  lcd.print("s ");

  lcd.setCursor(0, 0);

  // ========= FINISH LINE DETECTION =========
  static unsigned long blackStart = 0;
  unsigned long now = millis();

  if (leftBlack && middleBlack && rightBlack) {
    if (blackStart == 0) blackStart = now;
  } else {
    blackStart = 0;
  }

  if (blackStart != 0 && (now - blackStart >= FINISH_HOLD)) {
    stopCar();
    lcd.setCursor(0, 0);
    lcd.print("FINISH LINE   ");

    // Freeze final D & T
    lcd.setCursor(0, 1);
    lcd.print("D:");
    lcd.print(avgCm, 1);
    lcd.print("cm ");
    lcd.print("T:");
    lcd.print(elapsedSec, 1);
    lcd.print("s ");
    while (1);
  }

  // ========= NORMAL LINE FOLLOWING =========

  // Middle OK → go forward
  if (middleBlack) {
    moveForward();
    lcd.print("RUNNING       ");
    return; 
  }

  // Left sees black → correct RIGHT
  if (leftBlack) {
    lcd.print("COR R         ");
    while (!digitalRead(MIR)) turnRightSlow();
    return;
  }

  // Right sees black → correct LEFT
  if (rightBlack) {
    lcd.print("COR L         ");
    while (!digitalRead(MIR)) turnLeftSlow();
    return;
  }

  // LOST → sweep right
  sweepRight();
  lcd.print("SWEEP R       ");
}
