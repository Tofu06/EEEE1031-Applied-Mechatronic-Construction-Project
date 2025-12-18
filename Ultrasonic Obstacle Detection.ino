#include <LiquidCrystal.h>

// ================= LCD =================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ================= MOTOR PINS =================
const int IN1 = 13;
const int IN2 = 12;
const int ENA = A4;

const int IN3 = 1;
const int IN4 = 2;
const int ENB = A5;

// ================= SPEEDS =================
const int FWD_SPEED  = 130;
const int TURN_SPEED = 130;

// ================= ULTRASONIC SENSOR PINS =================
#define TRIG_PIN 11
#define ECHO_PIN 3

// ================= THRESHOLD =================
const int OBSTACLE_DISTANCE = 20; // cm

// ================= MOTOR FUNCTIONS =================
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, FWD_SPEED);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, FWD_SPEED);
  
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, TURN_SPEED);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, TURN_SPEED);
}

void turnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, TURN_SPEED);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, TURN_SPEED);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, FWD_SPEED);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, FWD_SPEED);
}

// ================= ULTRASONIC FUNCTION =================
float getDistanceCM() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  if (duration == 0) return -1;  // out of range

  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

// ====================== SETUP ======================
void setup() {
  lcd.begin(16, 2);
  lcd.print("Obstacle Avoider");
  delay(800);
  lcd.clear();

  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);
  delay(100);
}

// ====================== LOOP ======================
void loop() {

  float distance = getDistanceCM();

  // LCD Display
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  if (distance < 0) lcd.print("OUT   ");
  else {
    lcd.print(distance);
    lcd.print("cm   ");
  }

  // Serial Print
  Serial.print("Distance: ");
  if (distance < 0) Serial.println("Out of range");
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }

  // ===== LOGIC =====
  if (distance > 0 && distance <= OBSTACLE_DISTANCE) {
    // Obstacle detected
    lcd.setCursor(0, 1);
    lcd.print("Obstacle! Turn ");

    stopMotors();
    delay(200);
    backward();
    delay(20);

    // Choose LEFT or RIGHT
    turnLeft();        // turn left away from obstacle
    delay(400);        // adjust turning duration

    stopMotors();
    delay(100);

  } else {
    // Clear path → move forward
    lcd.setCursor(0, 1);
    lcd.print("Moving Forward ");

    forward();
  }
  delay(100);
}
