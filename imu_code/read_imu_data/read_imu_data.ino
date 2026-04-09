#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define IMU1_ADDR 0x68
#define IMU2_ADDR 0x69

void writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void readAccel(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);

  Wire.requestFrom(addr, (uint8_t)6);
  if (Wire.available() == 6) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Wake up both MPU6050s
  writeRegister(IMU1_ADDR, 0x6B, 0x00);
  writeRegister(IMU2_ADDR, 0x6B, 0x00);

  Serial.println("Both IMUs initialized.");
}

void loop() {
  int16_t ax1, ay1, az1;
  int16_t ax2, ay2, az2;

  readAccel(IMU1_ADDR, ax1, ay1, az1);
  readAccel(IMU2_ADDR, ax2, ay2, az2);

  Serial.print("IMU 1 (0x68): ");
  Serial.print("AX="); Serial.print(ax1);
  Serial.print(" AY="); Serial.print(ay1);
  Serial.print(" AZ="); Serial.println(az1);

  Serial.print("IMU 2 (0x69): ");
  Serial.print("AX="); Serial.print(ax2);
  Serial.print(" AY="); Serial.print(ay2);
  Serial.print(" AZ="); Serial.println(az2);

  Serial.println("-----");
  delay(500);
}