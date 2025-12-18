#include <SoftwareSerial.h>

// ================= MOTOR PINS =================
// Left motor
const int IN1 = 13;   // Left direction 1
const int IN2 = 12;   // Left direction 2
const int ENA = A4;   // Left enable (ON/OFF)

// Right motor
const int IN3 = 11;   // Right direction 1  (no longer on pin 1)
const int IN4 = 3;    // Right direction 2
const int ENB = A5;   // Right enable (ON/OFF)

// ================= BLUETOOTH PINS =================
const int BT_RX = A1;     // Arduino RX  (connect to BT TX)
const int BT_TX = A2;     // Arduino TX  (connect to BT RX)

SoftwareSerial BTserial(BT_RX, BT_TX);

int valSpeed = 255;

// ----------------- Motor Functions -----------------
void applyEnable(bool leftOn, bool rightOn) {
  digitalWrite(ENA, leftOn  ? HIGH : LOW);
  digitalWrite(ENB, rightOn ? HIGH : LOW);
}

void forward() {
  // left motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // right motor forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

void backward() {
  // left motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // right motor backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  // speed ctrls
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

void turnRight() {
  // left motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // right motor backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //speed ctrls
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

void turnLeft() {
  // left motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // right motor forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

void stopMotors() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void setSpeed(int s) {
  valSpeed = s;// car stops
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();
  Serial.println("BT Car Ready");
}

// ================= LOOP =================
void loop() {
  char cmd = 0;

  if (BTserial.available()) {
    cmd = BTserial.read();
  } else if (Serial.available()) {
    cmd = Serial.read();   // debug using Serial Monitor
  } else {
    return;
  }

  Serial.print("CMD: ");
  Serial.println(cmd);

  switch (cmd) {
    case 'F': case 'f': turnLeft(); break;
    case 'B': case 'b': turnRight(); break;
    case 'L': case 'l': forward(); break;
    case 'R': case 'r': backward(); break;
    case 'S': case 's': stopMotors(); break;

    // “speed” levels (app has a slider for speed so we used this)
    case '0': setSpeed(0); break;
    case '1': setSpeed(25); break;
    case '2': setSpeed(50); break;
    case '3': setSpeed(75); break;
    case '4': setSpeed(100); break;
    case '5': setSpeed(125); break;
    case '6': setSpeed(150); break;
    case '7': setSpeed(175); break;
    case '8': setSpeed(200); break;
    case '9': setSpeed(255); break;

    default:
      break;
  }
}
