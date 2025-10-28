#include <Arduino.h>

// --- Ultrasonic Pins ---
#define TRIG_F 4
#define ECHO_F 2
#define TRIG_L 16
#define ECHO_L 5
#define TRIG_R 23
#define ECHO_R 18
const float soundSpeed = 0.0343; // cm/us

// --- Motor Pins ---
#define LEFT_MOTOR_IN1 25
#define LEFT_MOTOR_IN2 27
#define RIGHT_MOTOR_IN1 26
#define RIGHT_MOTOR_IN2 33

// --- Motor Control ---
void leftMotorForward(int power) {
  analogWrite(LEFT_MOTOR_IN1, power);
  analogWrite(LEFT_MOTOR_IN2, 0);
}

void rightMotorForward(int power) {
  analogWrite(RIGHT_MOTOR_IN1, 0);
  analogWrite(RIGHT_MOTOR_IN2, power);
}

void leftMotorBackward(int power) {
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, power);
}

void rightMotorBackward(int power) {
  analogWrite(RIGHT_MOTOR_IN1, power);
  analogWrite(RIGHT_MOTOR_IN2, 0);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN1, 0);
  analogWrite(RIGHT_MOTOR_IN2, 0);
}

// --- Ultrasonic Function ---
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999; // No reading
  float distance = (duration * soundSpeed) / 2.0;
  if (distance > 500 || distance < 2) return 999; // Filter
  return distance;
}

// --- Motion Primitives ---
void moveForward(int leftPower, int rightPower) {
  leftMotorForward(leftPower);
  rightMotorForward(rightPower);
}

void smoothTurnRight(int basePower = 40) {
  leftMotorForward(basePower + 20);
  rightMotorForward(basePower - 20);
}

void smoothTurnLeft(int basePower = 40) {
  leftMotorForward(basePower - 20);
  rightMotorForward(basePower + 20);
}

void sharpTurnRight(int duration = 300) {
  leftMotorForward(60);
  rightMotorBackward(40);
  delay(duration);
  stopMotors();
}

void sharpTurnLeft(int duration = 300) {
  leftMotorBackward(40);
  rightMotorForward(60);
  delay(duration);
  stopMotors();
}

void moveBackward(int duration = 200) {
  leftMotorBackward(40);
  rightMotorBackward(40);
  delay(duration);
  stopMotors();
}

// --- Wall following variables ---
float prevRight = 15.0;

// --- Blockage detection variables ---
float prevFront = 0, prevLeft = 0, prevRightSensor = 0;
unsigned long lastChangeTime = 0;
const unsigned long BLOCKED_THRESHOLD = 1500; // 1.5s

// --- Maze Solving ---
void followMaze() {
  // Read sensors
  float frontDist = getDistance(TRIG_F, ECHO_F);
  float leftDist  = getDistance(TRIG_L, ECHO_L);
  float rightDist = getDistance(TRIG_R, ECHO_R);

  // Moving average filter for right sensor
  rightDist = (rightDist + prevRight * 3) / 4;
  prevRight = rightDist;

  // --- Blockage detection ---
  if (abs(frontDist - prevFront) < 1 && abs(leftDist - prevLeft) < 1 && abs(rightDist - prevRightSensor) < 1) {
    if (millis() - lastChangeTime > BLOCKED_THRESHOLD) {
      Serial.println("⛔ Blocked! Backing up and turning...");
      moveBackward(300);
      sharpTurnLeft(400);
      lastChangeTime = millis();
      return;
    }
  } else {
    lastChangeTime = millis();
  }

  prevFront = frontDist;
  prevLeft = leftDist;
  prevRightSensor = rightDist;

  Serial.print("F: "); Serial.print(frontDist);
  Serial.print(" | L: "); Serial.print(leftDist);
  Serial.print(" | R: "); Serial.println(rightDist);

  // --- Constants ---
  const float TARGET_DISTANCE = 15.0;
  const float FRONT_THRESHOLD = 20.0;
  const float SIDE_THRESHOLD = 30.0;
  const float MIN_DISTANCE = 8.0;

  // --- Emergency stop ---
  if (frontDist < MIN_DISTANCE || leftDist < MIN_DISTANCE || rightDist < MIN_DISTANCE) {
    Serial.println("⚠️ Too close! Backing up...");
    moveBackward(300);
    sharpTurnLeft(400);
    return;
  }

  // --- Dead end ---
  if (frontDist < FRONT_THRESHOLD && leftDist < 20 && rightDist < 20) {
    Serial.println("🚧 Dead end! Turning around...");
    moveBackward(300);
    sharpTurnLeft(600); // 180 degree turn
    return;
  }

  // --- Openings: right -> front -> left ---
  if (rightDist > SIDE_THRESHOLD) {
    Serial.println("➡️ Right opening! Turning right...");
    sharpTurnRight(350);
    return;
  }

  if (frontDist > FRONT_THRESHOLD) {
    Serial.println("⬆️ Forward is clear! Moving forward...");
    moveForward(45, 45);
    return;
  }

  if (leftDist > SIDE_THRESHOLD) {
    Serial.println("⬅️ Left opening! Turning left...");
    sharpTurnLeft(350);
    return;
  }

  // --- Wall following (PID-like) ---
  float error = TARGET_DISTANCE - rightDist;
  static float lastError = 0;
  float derivative = error - lastError;
  lastError = error;

  float Kp = 1.8;
  float Kd = 2.5;

  int adjustment = Kp * error + Kd * derivative;
  int baseSpeed = 45;
  int leftPower = constrain(baseSpeed + adjustment, 30, 65);
  int rightPower = constrain(baseSpeed - adjustment, 30, 65);

  Serial.print("Lp: "); Serial.print(leftPower);
  Serial.print(" | Rp: "); Serial.print(rightPower);
  Serial.print(" | Adj: "); Serial.println(adjustment);

  moveForward(leftPower, rightPower);
  delay(50);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  pinMode(LEFT_MOTOR_IN1, OUTPUT); pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT); pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  Serial.println("🤖 Maze Solver - Right Wall Follower");
  delay(1000);
}

// --- Loop ---
void loop() {
  followMaze();
}
