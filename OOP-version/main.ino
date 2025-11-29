#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include "Kalman.h"

// ================== KELAS IMU + BARO ==================
class IMUBaro {
public:
  IMUBaro() :
    gx_off(0), gy_off(0), gz_off(0),
    altitudeOffset(0), firstAltitudeRead(true),
    lastTime(0),
    rollAcc(0), pitchAcc(0),
    rollKalman(0), pitchKalman(0),
    rollComp(0), pitchComp(0),
    pressure(0), altitude(0),
    accY_lp(0), accZ_lp(0) {}

  bool begin() {
    Wire.begin();

    // Inisialisasi MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 tidak terdeteksi!");
      return false;
    }

    // Inisialisasi BMP280
    if (!bmp.begin(0x76)) {
      Serial.println("BMP280 tidak terdeteksi!");
      return false;
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_63);

    calibrateGyro();
    initKalman();
    lastTime = micros();

    return true;
  }

  // Dipanggil di loop() untuk update semua perhitungan
  void update() {
    // Hitung dt
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    // Baca raw data IMU
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    // Low-pass filter (kalau mau pakai nantinya)
    accY_lp = 0.96f * accY_lp + 0.04f * accY;
    accZ_lp = 0.96f * accZ_lp + 0.04f * accZ;

    // Sudut dari accelerometer
    rollAcc  = atan2(accY, accZ) * 180.0 / PI;
    pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;

    // Konversi gyro ke derajat/detik
    float gyroXrate = (gyroX - gx_off) / 131.0;
    float gyroYrate = (gyroY - gy_off) / 131.0;

    // Kalman filter
    rollKalman  = kalmanRoll.getAngle(rollAcc,  gyroXrate, dt);
    pitchKalman = kalmanPitch.getAngle(pitchAcc, gyroYrate, dt);

    // Complementary filter (jika mau dipakai)
    rollComp  = 0.995f * (rollComp  + gyroXrate * dt) + 0.005f * rollAcc;
    pitchComp = 0.995f * (pitchComp + gyroYrate * dt) + 0.005f * pitchAcc;

    // BMP280
    pressure = bmp.readPressure() / 100.0;   // hPa

    float altitudeRaw = bmp.readAltitude(1013.25);
    if (firstAltitudeRead) {
      altitudeOffset = altitudeRaw;
      firstAltitudeRead = false;
    }
    altitude = altitudeRaw - altitudeOffset;
  }

  // Getter sudut & ketinggian
  float getRollKalman()  const { return rollKalman; }
  float getPitchKalman() const { return pitchKalman; }
  float getRollComp()    const { return rollComp; }
  float getPitchComp()   const { return pitchComp; }
  float getAltitude()    const { return altitude; }
  float getPressure()    const { return pressure; }

private:
  MPU6050 mpu;
  Adafruit_BMP280 bmp;
  Kalman kalmanRoll;
  Kalman kalmanPitch;

  // Raw data sensor
  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;

  // Offset gyro
  float gx_off, gy_off, gz_off;

  // Variabel sudut
  float rollAcc, pitchAcc;
  float rollKalman, pitchKalman;
  float rollComp, pitchComp;

  // Variabel barometer
  float pressure;
  float altitude;
  float altitudeOffset;
  bool  firstAltitudeRead;

  // Low-pass filter akselerometer (opsional)
  float accY_lp, accZ_lp;

  // Timing
  unsigned long lastTime;

  void calibrateGyro() {
    long sumX = 0, sumY = 0, sumZ = 0;
    const int N = 500;
    Serial.println("Kalibrasi gyro, harap diamkan modul...");
    for (int i = 0; i < N; i++) {
      mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;
      delay(3);
    }
    gx_off = sumX / (float)N;
    gy_off = sumY / (float)N;
    gz_off = sumZ / (float)N;

    Serial.println("Kalibrasi gyro selesai.");
  }

  void initKalman() {
    // Baca awal untuk inisialisasi Kalman
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
    rollAcc  = atan2(accY, accZ) * 180.0 / PI;
    pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;

    kalmanRoll.setAngle(rollAcc);
    kalmanPitch.setAngle(pitchAcc);

    rollComp  = rollAcc;
    pitchComp = pitchAcc;
  }
};

// ================== OBJEK GLOBAL ==================
IMUBaro imuBaro;

// Target sudut
float targetRoll  = 0.0;
float targetPitch = 0.0;

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  while (!Serial) {;}

  Serial.println("Mulai inisialisasi sensor...");

  if (!imuBaro.begin()) {
    Serial.println("Inisialisasi sensor gagal. Cek koneksi hardware.");
    while (1);
  }

  Serial.println("Sensor siap...");
}

// ================== FUNGSI INPUT ==================
void handleKeyboardInput() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'w') targetPitch += 5;
    if (cmd == 's') targetPitch -= 5;
    if (cmd == 'a') targetRoll  += 5;
    if (cmd == 'd') targetRoll  -= 5;

    // Batas aman
    if (targetRoll > 90) targetRoll = 90;
    if (targetRoll < -90) targetRoll = -90;
    if (targetPitch > 90) targetPitch = 90;
    if (targetPitch < -90) targetPitch = -90;
  }
}

// ================== LOOP ==================
void loop() {
  handleKeyboardInput();

  // Update semua perhitungan sensor
  imuBaro.update();

  float rollKal   = imuBaro.getRollKalman();
  float pitchKal  = imuBaro.getPitchKalman();
  float altitude  = imuBaro.getAltitude();
  // float pressure = imuBaro.getPressure(); // kalau mau dipakai

  // Kirim data via Serial
  Serial.print("RollKalman:");  Serial.print(rollKal);
  Serial.print(",PitchKalman:"); Serial.print(pitchKal);
  Serial.print(",Altitude:");   Serial.print(altitude);
  Serial.print(" ThredHoldRoll :"); Serial.println(targetRoll);

  // Alert system
  float rollThreshold = targetRoll;
  if (abs(rollKal) >= rollThreshold) {
    Serial.println("⚠ ROLL MELEBIHI BATAS!");
  }

  delay(20); // ~50 Hz
}
