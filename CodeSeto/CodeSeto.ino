#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include "Kalman.h" 

//HardwareSerial Telemetry(2); // UART2

MPU6050 mpu;
Adafruit_BMP280 bmp;

#define RX2_PIN 16
#define TX2_PIN 17

// Variabel sensor
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float rollAcc, pitchAcc;
float rollKalman, pitchKalman;
float rollComp, pitchComp;
float pressure, altitude;

float gx_off = 0, gy_off = 0, gz_off = 0;

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

float altitudeOffset = 0 ;
bool firstAltitudeRead = true;

// Kalman filter instance
Kalman kalmanRoll;
Kalman kalmanPitch;

float targetRoll = 0;
float targetPitch = 0;

unsigned long lastTime;
float dt;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terdeteksi!");
    while (1);
  }

  // initialize BMP280
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
  // membaca data awal untuk inisialisasi kalman
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  rollAcc = atan2(accY, accZ) * 180 / PI;
  pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  kalmanRoll.setAngle(rollAcc);
  kalmanPitch.setAngle(pitchAcc);

  rollComp = rollAcc;
  pitchComp = pitchAcc;

  lastTime = micros();

  //Telemetry.begin(57600, SERIAL_8N1, 16, 17); // RX=16 , TX=17

  //Serial.println("Sensor siap...");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'w') targetPitch += 5;
    if (cmd == 's') targetPitch -= 5;
    if (cmd == 'a') targetRoll  += 5;
    if (cmd == 'd') targetRoll  -= 5;

    // batas aman (opsional)
    if (targetRoll > 90) targetRoll = 90;
    if (targetRoll < -90) targetRoll = -90;
    if (targetPitch > 90) targetPitch = 90;
    if (targetPitch < -90) targetPitch = -90;

    //Serial.print(" | TargetPitch = ");
    //Serial.println(targetPitch);
  }
  // hitung dt


  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // ambil data raw
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  static float accY_lp = 0, accZ_lp = 0;
  accY_lp = 0.96 * accY_lp + 0.04 * accY;
  accZ_lp = 0.96 * accZ_lp + 0.04 * accZ;

  // hitung angle dari accelerometer
  rollAcc  = atan2(accY, accZ) * 180 / PI;
  pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Konversi gyro ke derajat/detik
  float gyroXrate = (gyroX - gx_off) / 131.0;
  float gyroYrate = (gyroY - gy_off) / 131.0;


  // Kalman filter
  rollKalman  = kalmanRoll.getAngle(rollAcc,  gyroXrate, dt);
  pitchKalman = kalmanPitch.getAngle(pitchAcc, gyroYrate, dt);

  // Complementary filter (α = 0.98)
  rollComp  = 0.995 * (rollComp  + gyroXrate * dt) + 0.005 * rollAcc;
  pitchComp = 0.995 * (pitchComp + gyroYrate * dt) + 0.005 * pitchAcc;

  // BMP280
  pressure = bmp.readPressure() / 100.0;   // hPa

  float altitudeRaw = bmp.readAltitude(1013.25);

  if (firstAltitudeRead) {
    altitudeOffset = altitudeRaw;
    firstAltitudeRead = false;
}

altitude = altitudeRaw - altitudeOffset;

  // KIRIM DATA TELEMETRY
  //Telemetry.print("RollKalman:"); Telemetry.print(rollKalman);
  //Telemetry.print(",PitchKalman:"); Telemetry.print(pitchKalman);
  //Telemetry.print(",RollComp:"); Telemetry.print(rollComp);
  //Telemetry.print(",PitchComp:"); Telemetry.print(pitchComp);
  //Telemetry.print(",Altitude:"); Telemetry.print(altitude);
  //Telemetry.print(",Pressure:"); Telemetry.println(pressure);


  // KIRIM DATA SERIAL
  Serial.print("RollKalman:"); Serial.print(rollKalman);
  Serial.print(",PitchKalman:"); Serial.print(pitchKalman);
  //Serial.print(",RollComp:"); Serial.print(rollComp);
  //Serial.print(",PitchComp:"); Serial.print(pitchComp);
  Serial.print(",Altitude:"); Serial.print(altitude);
  //Serial.print(",Pressure:"); Serial.println(pressure);
  Serial.print(" ThredHoldRoll :"); Serial.println(targetRoll);

  // SIMPAN DATA 
  

  // ---------------- ALERT SYSTEM ----------------
  float rollThreshold = targetRoll; // batas derajat roll

  if (abs(rollKalman) >= rollThreshold) {
  Serial.println("⚠ ROLL MELEBIHI BATAS!");
  }

  delay(20); // 50Hz
}
