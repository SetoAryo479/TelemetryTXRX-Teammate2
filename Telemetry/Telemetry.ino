#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include "kalman.h"  

HardwareSerial Telemetry(2); // UART2 (TX2,RX2)
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

// Kalman filter instance
Kalman kalmanRoll;
Kalman kalmanPitch;

unsigned long lastTime;
float dt;

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
  Telemetry.begin(57600, SERIAL_8N1, 16, 17); // RX2=16, TX2=17 untuk SiK Telemetry

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
}

void loop() {
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

  rollComp  = 0.995 * (rollComp  + gyroXrate * dt) + 0.005 * rollAcc;
  pitchComp = 0.995 * (pitchComp + gyroYrate * dt) + 0.005 * pitchAcc;

  pressure = bmp.readPressure() / 100.0;

  float altitudeRaw = bmp.readAltitude(1013.25);
  if (firstAltitudeRead) {
    altitudeOffset = altitudeRaw;
    firstAltitudeRead = false;
  }
  altitude = altitudeRaw - altitudeOffset;

  // format data telemetry
  String data =
    String("RollKalman:") + rollKalman +
    ",PitchKalman:" + pitchKalman +
    ",RollComp:" + rollComp +
    ",PitchComp:" + pitchComp +
    ",Altitude:" + altitude +
    ",Pressure:" + pressure;

  // kirim ke Serial + Telemetry
  Serial.println(data);
  Telemetry.println(data);      // 🔥 dikirim ke receiver telemetry

  delay(20); // 50Hz
}
