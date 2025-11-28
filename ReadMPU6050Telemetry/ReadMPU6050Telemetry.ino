#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variabel sensor
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// Serial2 (Telemetry)
#define TX2 17
#define RX2 16

void setup() {
  Serial.begin(115200);
  Serial2.begin(57600, SERIAL_8N1, RX2, TX2);  // Telemetry

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 Connected");
  } else {
    Serial.println("MPU6050 Failed!");
    while (1);
  }
}

void loop() {
  // Baca akselerometer & gyroscope
  mpu.getAcceleration(&accX, &accY, &accZ);
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);

  // Kirim ke serial monitor
  Serial.print("ACC: ");
  Serial.print(accX); Serial.print(", ");
  Serial.print(accY); Serial.print(", ");
  Serial.print(accZ);

  Serial.print(" | GYRO: ");
  Serial.print(gyroX); Serial.print(", ");
  Serial.print(gyroY); Serial.print(", ");
  Serial.println(gyroZ);

  // Kirim ke telemetry (Serial2)
  Serial2.print("ACC:");
  Serial2.print(accX); Serial2.print(",");
  Serial2.print(accY); Serial2.print(",");
  Serial2.print(accZ);

  Serial2.print(";GYRO:");
  Serial2.print(gyroX); Serial2.print(",");
  Serial2.print(gyroY); Serial2.print(",");
  Serial2.println(gyroZ);

  delay(50);
}
