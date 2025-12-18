#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>

// ================= LCD PINS=================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// ================= MPU6050 PINS=================
Adafruit_MPU6050 mpu;
// ================= MOTOR PINS =================
const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = 1;
const int IN4 = 2;
// ================= SPEEDS CONTROLS =================
const int FWD_SPEED  = 255;
const int SPIN_SPEED = 150;
// ================= ANGLE CONSTANTS =================
const float PITCH_OFFSET = 85.0;   // your board offset
const float RAMP_UP_THRESHOLD    = 15.0;
const float RAMP_DOWN_THRESHOLD  = -15.0;
const float TOP_FLAT_THRESHOLD    = 10.0;
const float BOTTOM_FLAT_THRESHOLD = 10.0;

const unsigned long TOP_FLAT_HOLD_MS    = 120;
const unsigned long BOTTOM_FLAT_HOLD_MS = 200;

const unsigned long SPIN_TIMEOUT_MS = 5000;
// ================= STATE MACHINE =================
enum State { ASCEND, HOLD_TOP, SPIN, DESCEND, DONE };
State state = ASCEND;

bool hasClimbedRamp = false;
bool hasDescendedRamp = false;

unsigned long holdStart = 0;
unsigned long spinStart = 0;
unsigned long lastTime  = 0;

float yawDeg = 0;
float pitchFiltered = 0;
bool pitchInit = false;

// ===== store maximum ramp angle =====
float maxRampAngle = 0.0;

// ================= MOTOR FUNCTIONS =================
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, FWD_SPEED);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, FWD_SPEED);
}

void backward() {
  digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW);
  analogWrite(ENA, FWD_SPEED);

  digitalWrite(IN3, HIGH);  digitalWrite(IN4, LOW);
  analogWrite(ENB, FWD_SPEED);
}

void spinInPlace() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  analogWrite(ENA, SPIN_SPEED);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, SPIN_SPEED);
}
// ================= ANGLE HELPERS =================
float computeAccelPitch(sensors_event_t &a) {
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  return pitch + PITCH_OFFSET;
}

// ====================== SETUP ======================
void setup() {
  lcd.begin(16, 2);
  lcd.print("MPU Ramp Bot");
  delay(800);
  lcd.clear();

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopMotors();

  if (!mpu.begin()) {
    lcd.print("MPU ERROR");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  lastTime = millis();
}

// ====================== LOOP ======================
void loop() {

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  // ---- Accelerometer pitch ----
  float pitchAccel = computeAccelPitch(accel);

  // ---- Gyro pitch rate (choose correct axis depending on board orientation) ----
  float gyroPitchRate = -gyro.gyro.y * 180.0 / PI;

  // ---- Complementary filter ----
  const float alpha = 0.98;

  if (!pitchInit) {
    pitchFiltered = pitchAccel;
    pitchInit = true;
  } else {
    float pitchGyroIntegrated = pitchFiltered + gyroPitchRate * dt;
    pitchFiltered = alpha * pitchGyroIntegrated + (1 - alpha) * pitchAccel;
  }

  // ---- Gyro for yaw (spin) ----
  float gx = gyro.gyro.x * 180.0 / PI;
  float gy = gyro.gyro.y * 180.0 / PI;
  float gz = gyro.gyro.z * 180.0 / PI;

  // ================= LCD PITCH DISPLAY =================
  lcd.setCursor(0, 0);
  lcd.print("P:");
  lcd.print(pitchFiltered, 1);
  lcd.print("     ");
  lcd.setCursor(0, 1);
  float p = pitchFiltered;

  // ======================= STATE MACHINE =======================

  switch (state) {
    // -------------------- ASCEND PHASE -------------------------
    case ASCEND:
      lcd.print("ASCEND       ");
      forward();

      // Track maximum angle while climbing
      if (p > maxRampAngle) maxRampAngle = p;

      // Detect uphill
      if (p > RAMP_UP_THRESHOLD) {
        hasClimbedRamp = true;
      }

      // Confirm top when pitch returns near flat for a bit
      static unsigned long topFlatStart = 0;
      if (hasClimbedRamp && fabs(p) < TOP_FLAT_THRESHOLD) {
        if (topFlatStart == 0) topFlatStart = now;

        if (now - topFlatStart >= TOP_FLAT_HOLD_MS) {
          delay(350);
          stopMotors();
          holdStart = now;
          state = HOLD_TOP;
        }
      } else {
        topFlatStart = 0;
      }
      break;
      
    // -------------- HOLD 4 SECONDS ON TOP ------------------
    case HOLD_TOP:
      lcd.print("HOLD 4s      ");
      stopMotors();

      if (now - holdStart >= 4000) {
        yawDeg = 0;
        spinStart = now;
        state = SPIN;
      }
      break;

// -------------------- SPIN 360 DEGREES -----------------------
    case SPIN: {
      // pick axis with strongest spin
      float yawRate = gx;
      if (fabs(gy) > fabs(yawRate)) yawRate = gy;
      if (fabs(gz) > fabs(yawRate)) yawRate = gz;

      yawDeg += yawRate * dt;

      lcd.print("SPN:");
      lcd.print(yawDeg, 0);
      lcd.print("   ");

      spinInPlace();

      bool angleDone   = (fabs(yawDeg) >= 360.0);
      bool timeoutDone = (now - spinStart >= SPIN_TIMEOUT_MS);

      if (angleDone || timeoutDone) {
        stopMotors();
        delay(300);
        hasDescendedRamp = false;
        state = DESCEND;
      }
      break;
    }

//   -------------------  DESCEND RAMP  -------------------------
    case DESCEND:
      lcd.print("DESCEND      ");
      backward();

      if (p < RAMP_DOWN_THRESHOLD) {
        hasDescendedRamp = true;
      }

      static unsigned long bottomFlatStart = 0;
      if (hasDescendedRamp && fabs(p) < BOTTOM_FLAT_THRESHOLD) {
        if (bottomFlatStart == 0) bottomFlatStart = now;

        if (now - bottomFlatStart >= BOTTOM_FLAT_HOLD_MS) {
          stopMotors();
          state = DONE;
        }
      } else {
        bottomFlatStart = 0;
      }
      break;
      
// ---------------------    DONE    --------------------------
    case DONE:
      stopMotors();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("FINISHED");

      lcd.setCursor(0,1);
      lcd.print("Max:");
      lcd.print(maxRampAngle, 1);
      lcd.print(" deg");

      while (1);  // freeze forever
      break;
  }
  delay(10);
}
