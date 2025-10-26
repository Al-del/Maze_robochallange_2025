#include <Wire.h>
#include <math.h>

#define MPU9250_ADDR 0x68
#define GYRO_SENS 32.8 // Gyro full scale ±1000 °/s → 32.8 LSB/(°/s)

float yaw = 0;
unsigned long prevTime = 0;

// Gyro offsets
float gz_offset = -114;

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
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
}

// --- Calibrate gyro Z-axis ---
void calibrateGyroZ() {
  uint8_t data[6];
  int32_t gz_sum = 0;
  for(int i=0; i<500; i++){
    readRegisters(0x43, 6, data); // gx, gy, gz
    gz_sum += (int16_t)((data[4]<<8)|data[5]);
    delay(2);
  }
  gz_offset = gz_sum / 500.0;
  Serial.println("✅ Gyro Z calibration done!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(17, 14); // SDA=17, SCL=14
  delay(1000);

  // Wake up MPU-9250
  writeRegister(0x6B, 0x00);
  delay(100);

  // Set gyro ±1000°/s
  writeRegister(0x1B, 0x10);

  prevTime = millis();
}

void loop() {
  static bool firstLoop = true;
  if(firstLoop){ calibrateGyroZ(); firstLoop=false; }

  uint8_t rawData[14];
  readRegisters(0x3B, 14, rawData);

  // Gyro Z-axis
  float gz = ((int16_t)((rawData[12]<<8)|rawData[13]) - gz_offset) / GYRO_SENS;

  unsigned long currTime = millis();
  float dt = (currTime - prevTime)/1000.0;
  prevTime = currTime;

  // Integrate gz to get yaw
  yaw += gz * dt;

  Serial.print("Yaw (Z-axis): ");
  Serial.println(yaw, 2);

  delay(20);
}
