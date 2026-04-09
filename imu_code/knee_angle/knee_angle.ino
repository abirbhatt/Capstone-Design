#include <Wire.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define THIGH_IMU 0x68
#define SHIN_IMU  0x69

// MPU6050 registers
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// Filter settings
float alpha = 0.98;   // complementary filter weight

// Orientation values
float thighPitch = 0.0;
float shinPitch  = 0.0;

float zeroOffset = 0.0;   // calibrated when leg is straight

unsigned long lastTime = 0;

struct IMUData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

void writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool readMPU6050(uint8_t addr, IMUData &data) {
  Wire.beginTransmission(addr);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(addr, (uint8_t)14);
  if (Wire.available() < 14) {
    return false;
  }

  data.ax = (Wire.read() << 8) | Wire.read();
  data.ay = (Wire.read() << 8) | Wire.read();
  data.az = (Wire.read() << 8) | Wire.read();

  // skip temperature
  Wire.read();
  Wire.read();

  data.gx = (Wire.read() << 8) | Wire.read();
  data.gy = (Wire.read() << 8) | Wire.read();
  data.gz = (Wire.read() << 8) | Wire.read();

  return true;
}

float accelPitchDegrees(const IMUData &d) {
  // Pitch estimate from accel only
  // You may need to change axes depending on your IMU orientation
  float ax = (float)d.ax;
  float ay = (float)d.ay;
  float az = (float)d.az;

  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  return pitch;
}

float gyroYDegreesPerSecond(const IMUData &d) {
  // MPU6050 gyro sensitivity at default ±250 deg/s = 131 LSB/(deg/s)
  return (float)d.gy / 131.0;
}

void initMPU(uint8_t addr) {
  writeRegister(addr, PWR_MGMT_1, 0x00); // wake up
  delay(100);
}

void calibrateZeroOffset() {
  Serial.println("Calibrating zero offset...");
  Serial.println("Keep leg straight and still.");

  float sum = 0.0;
  const int samples = 200;

  for (int i = 0; i < samples; i++) {
    IMUData thighData, shinData;
    if (readMPU6050(THIGH_IMU, thighData) && readMPU6050(SHIN_IMU, shinData)) {
      float thighAccelPitch = accelPitchDegrees(thighData);
      float shinAccelPitch  = accelPitchDegrees(shinData);
      sum += (shinAccelPitch - thighAccelPitch);
    }
    delay(10);
  }

  zeroOffset = sum / samples;

  Serial.print("Zero offset = ");
  Serial.println(zeroOffset);
  Serial.println("Calibration done.");
}

void scanI2C() {
  Serial.println("Scanning I2C...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  scanI2C();

  initMPU(THIGH_IMU);
  initMPU(SHIN_IMU);

  // initialize filter with accel-based estimate
  IMUData thighData, shinData;
  if (readMPU6050(THIGH_IMU, thighData)) {
    thighPitch = accelPitchDegrees(thighData);
  }
  if (readMPU6050(SHIN_IMU, shinData)) {
    shinPitch = accelPitchDegrees(shinData);
  }

  delay(500);
  calibrateZeroOffset();

  lastTime = micros();
}

void loop() {
  IMUData thighData, shinData;

  if (!readMPU6050(THIGH_IMU, thighData)) {
    Serial.println("Failed to read thigh IMU");
    delay(100);
    return;
  }

  if (!readMPU6050(SHIN_IMU, shinData)) {
    Serial.println("Failed to read shin IMU");
    delay(100);
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // accel-based pitch
  float thighAccelPitch = accelPitchDegrees(thighData);
  float shinAccelPitch  = accelPitchDegrees(shinData);

  // gyro rate
  float thighGyroRate = gyroYDegreesPerSecond(thighData);
  float shinGyroRate  = gyroYDegreesPerSecond(shinData);

  // complementary filter
  thighPitch = alpha * (thighPitch + thighGyroRate * dt) + (1.0 - alpha) * thighAccelPitch;
  shinPitch  = alpha * (shinPitch  + shinGyroRate  * dt) + (1.0 - alpha) * shinAccelPitch;

  // relative knee angle
  float kneeAngle = fabs((shinPitch - thighPitch) - zeroOffset);

  Serial.print("Thigh: ");
  Serial.print(thighPitch, 2);
  Serial.print(" deg\t");

  Serial.print("Shin: ");
  Serial.print(shinPitch, 2);
  Serial.print(" deg\t");

  Serial.print("Knee: ");
  Serial.print(kneeAngle, 2);
  Serial.println(" deg");

  delay(20);
}