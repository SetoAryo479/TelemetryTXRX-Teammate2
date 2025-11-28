#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include "Kalman.h"  

HardwareSerial Telemetry(2);
MPU6050 mpu;
Adafruit_BMP280 bmp;

// Variabel sensor
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float rollAcc, pitchAcc;
float rollKalman, pitchKalman;
float rollComp, pitchComp;
float pressure, altitude;

float gx_off = 0, gy_off = 0, gz_off = 0;

float altitudeOffset = 0;
bool firstAltitudeRead = true;

Kalman kalmanRoll;
Kalman kalmanPitch;

unsigned long lastTime;
float dt;

// ===== Target sudut dari keyboard =====
float targetRoll = 0;
float targetPitch = 0;

void calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    delay(3);
  }
  gx_off = sumX / 500.0;
  gy_off = sumY / 500.0;
  gz_off = sumZ / 500.0;
}

void setup() {
  Serial.begin(115200);
  Telemetry.begin(57600, SERIAL_8N1, 16, 17);

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terdeteksi!");
    while (1);
  }

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 tidak terdeteksi!");
    while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  calibrateGyro();

  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  rollAcc = atan2(accY, accZ) * 180 / PI;
  pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  kalmanRoll.setAngle(rollAcc);
  kalmanPitch.setAngle(pitchAcc);

  rollComp = rollAcc;
  pitchComp = pitchAcc;

  lastTime = micros();

  Serial.println("Sensor + Telemetry siap...");
  Serial.println("Kontrol keyboard: w(+pitch), s(-pitch), a(+roll), d(-roll)");
}

void loop() {

  // ======== Kontrol w s a d dari laptop ========
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'w') targetPitch += 5;
    if (cmd == 's') targetPitch -= 5;
    if (cmd == 'a') targetRoll  += 5;
    if (cmd == 'd') targetRoll  -= 5;

    // batas aman (opsional)
    if (targetRoll > 60) targetRoll = 60;
    if (targetRoll < -60) targetRoll = -60;
    if (targetPitch > 60) targetPitch = 60;
    if (targetPitch < -60) targetPitch = -60;

    Serial.print("TargetRoll = ");
    Serial.print(targetRoll);
    Serial.print(" | TargetPitch = ");
    Serial.println(targetPitch);
  }

  // ======== Perhitungan IMU ========
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  rollAcc  = atan2(accY, accZ) * 180 / PI;
  pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  float gyroXrate = (gyroX - gx_off) / 131.0;
  float gyroYrate = (gyroY - gy_off) / 131.0;

  rollKalman  = kalmanRoll.getAngle(rollAcc,  gyroXrate, dt);
  pitchKalman = kalmanPitch.getAngle(pitchAcc, gyroYrate, dt);

  pressure = bmp.readPressure() / 100.0;

  float altitudeRaw = bmp.readAltitude(1013.25);
  if (firstAltitudeRead) {
    altitudeOffset = altitudeRaw;
    firstAltitudeRead = false;
  }
  altitude = altitudeRaw - altitudeOffset;

  // ======== Data Telemetry ========
  String data =
    String("RollKalman:") + rollKalman +
    ",PitchKalman:" + pitchKalman +
    ",Altitude:" + altitude +
    ",Pressure:" + pressure +
    ",TargetRoll:" + targetRoll +
    ",TargetPitch:" + targetPitch;

  Serial.println(data);
  Telemetry.println(data);

  delay(20);
}
