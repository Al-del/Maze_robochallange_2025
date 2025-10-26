#include <Wire.h>
#include <math.h>

// --- MPU9250 Setup ---
#define MPU9250_ADDR 0x68
#define GYRO_SENS 32.8 // ±1000 °/s -> 32.8 LSB/(°/s)
float yaw = 0;
unsigned long prevTime = 0;
float gz_offset = -114;

// --- Motor Pins ---
#define LEFT_MOTOR_IN1 25
#define LEFT_MOTOR_IN2 27

// --- I2C Pins ---
#define SDA 17
#define SCL 14

// --- Functions for MPU9250 ---
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, count);
  for (uint8_t i = 0; i < count; i++) dest[i] = Wire.read();
}

void calibrateGyroZ() {
  uint8_t data[6];
  int32_t gz_sum = 0;
  Serial.println("Calibrating Gyro Z...");
  for (int i = 0; i < 500; i++) {
    readRegisters(0x43, 6, data);
    gz_sum += (int16_t)((data[4] << 8) | data[5]);
    delay(2);
  }
  gz_offset = gz_sum / 500.0;
  Serial.println("✅ Gyro Z calibration done!");
}

// --- Motor control ---
void LeftMotorPower(int power) {
  power = constrain(power, -255, 255);
  if (power > 0) {
    analogWrite(LEFT_MOTOR_IN1, power);
    analogWrite(LEFT_MOTOR_IN2, 0);
  } else if (power < 0) {
    analogWrite(LEFT_MOTOR_IN1, 0);
    analogWrite(LEFT_MOTOR_IN2, -power);
  } else {
    analogWrite(LEFT_MOTOR_IN1, 0);
    analogWrite(LEFT_MOTOR_IN2, 0);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  delay(1000);

  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);

  // Initialize MPU
  writeRegister(0x6B, 0x00);  // Wake up
  delay(100);
  writeRegister(0x1B, 0x10);  // ±1000°/s
  delay(100);

  calibrateGyroZ();
  prevTime = millis();
  Serial.println("Starting rotation...");
}

void loop() {
  uint8_t rawData[14];
  readRegisters(0x3B, 14, rawData);

  float gz = ((int16_t)((rawData[12] << 8) | rawData[13]) - gz_offset) / GYRO_SENS;

  unsigned long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  yaw += gz * dt;
  float absYaw = fabs(yaw);

  Serial.print("Yaw: ");
  Serial.print(yaw, 2);
  Serial.print(" | AbsYaw: ");
  Serial.println(absYaw, 2);

  // --- Rotate until ±90 degrees reached ---
  if (absYaw < 90.0) {
    // Smooth slowdown: lower PWM as we approach target
    int baseSpeed = 50; // normal slow speed
    int minSpeed = 25;  // near stop
    float slowZone = 20.0; // degrees before stopping

    int pwm = baseSpeed;
    if (absYaw > (90.0 - slowZone)) {
      pwm = map(absYaw, 90.0 - slowZone, 90.0, baseSpeed, minSpeed);
    }
    pwm = constrain(pwm, minSpeed, baseSpeed);

    LeftMotorPower(pwm);
  } else {
    LeftMotorPower(0);
    Serial.println("🛑 Target reached: ±90°");
    while (1); // Stop permanently
  }

  delay(20);
}
