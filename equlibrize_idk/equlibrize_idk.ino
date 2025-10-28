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
  if (duration == 0) return -1; // Sensor timeout
  return (duration * soundSpeed) / 2.0;
}

// --- Move Forward with PID and Spike Protection ---
void moveForwardDistance(float targetDistanceCM, int baseLeft, int baseRight) {
  float startDist = getDistance(TRIG_F, ECHO_F);
  if (startDist < 0) {
    Serial.println("❌ Front ultrasonic error at start!");
    return;
  }

  Serial.print("Start distance: "); Serial.println(startDist);

  float prevError = 0;
  float integral = 0;
  float lastCorrection = 0;

  // PID constants
  float Kp = 1.0;
  float Ki = 0.05;
  float Kd = 0.2;
  float maxCorrection = 20;

  // Spike protection
  float maxErrorJump = 15; // ignore spikes bigger than this
  int freezeCycles = 3;     // cycles to freeze after spike
  int freezeCounter = 0;

  while (true) {
    float currDist = getDistance(TRIG_F, ECHO_F);
    if (currDist < 0) continue;

    float traveled = startDist - currDist;
    if (traveled >= targetDistanceCM) break;

    float leftDist = getDistance(TRIG_L, ECHO_L);
    float rightDist = getDistance(TRIG_R, ECHO_R);
    if (leftDist < 0 || rightDist < 0) continue;

    float error = leftDist - rightDist;

    // Spike detection
    if (abs(error - prevError) > maxErrorJump) {
      freezeCounter = freezeCycles;
    }

    float correction;
    if (freezeCounter > 0) {
      // keep previous correction
      correction = lastCorrection;
      freezeCounter--;
    } else {
      // normal PID
      integral += error;
      float derivative = error - prevError;
      correction = Kp * error + Ki * integral + Kd * derivative;
      correction = constrain(correction, -maxCorrection, maxCorrection);
      lastCorrection = correction;
    }

    prevError = error;

    int adjLeft = constrain(baseLeft - correction, 0, 255);
    int adjRight = constrain(baseRight + correction, 0, 255);

    leftMotorForward(adjLeft);
    rightMotorForward(adjRight);

    Serial.print("Traveled: "); Serial.print(traveled);
    Serial.print(" | L: "); Serial.print(adjLeft);
    Serial.print(" R: "); Serial.println(adjRight);

    delay(50);
  }

  stopMotors();
  Serial.println("✅ Target reached!");
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  pinMode(LEFT_MOTOR_IN1, OUTPUT); pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT); pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  Serial.println("✅ System ready!");
  delay(1000);
}

void loop() {
  moveForwardDistance(300, 30, 30); // Move 300cm, base speed 30 each
  delay(3000);
}
