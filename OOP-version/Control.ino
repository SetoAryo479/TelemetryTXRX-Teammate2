#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================== KONFIGURASI PIN & SERIAL ==================
#define I2C_SDA 21
#define I2C_SCL 22

#define TELEMETRY_RX 16   // RX ESP32 (ke TX modul)
#define TELEMETRY_TX 17   // TX ESP32 (ke RX modul)

// ================== LOOP TIMING ==================
const unsigned long LOOP_INTERVAL_US = 10000UL; // 100 Hz

// ================== KELAS KALMAN 1D ==================
class Kalman1D {
public:
  Kalman1D(float q = 0.001f, float r = 0.03f, float p0 = 1.0f, float x0 = 0.0f)
    : Q(q), R(r), P(p0), x(x0), initialized(false) {}

  void setAngle(float angle) {
    x = angle;
    initialized = true;
  }

  float getAngle() const {
    return x;
  }

  // rate: gyro rate (deg/s), measAngle: sudut dari accelerometer (deg)
  float update(float rate, float measAngle, float dt) {
    if (!initialized) {
      // kalau belum di-init, langsung set dari measurement
      setAngle(measAngle);
      return x;
    }

    // Prediksi
    float x_pred = x + rate * dt;
    float P_pred = P + Q;

    // Update
    float K = P_pred / (P_pred + R);
    x = x_pred + K * (measAngle - x_pred);
    P = (1.0f - K) * P_pred;

    return x;
  }

private:
  float Q;  // process noise
  float R;  // measurement noise
  float P;  // error covariance
  float x;  // state (angle)
  bool  initialized;
};

// ================== KELAS MPU6050 ==================
class MPU6050 {
public:
  MPU6050(uint8_t address = 0x68)
    : _addr(address),
      _roll(0.0f),
      _pitch(0.0f) {}

  void begin() {
    // Wake up MPU6050
    writeByte(0x6B, 0x00); // PWR_MGMT_1: clear sleep bit
    // Sample rate: 1kHz / (1 + 9) = 100 Hz
    writeByte(0x19, 0x09); // SMPLRT_DIV
    // DLPF config
    writeByte(0x1A, 0x03); // CONFIG
    // Gyro full scale ±250 deg/s
    writeByte(0x1B, 0x00); // GYRO_CONFIG
    // Accel full scale ±2g
    writeByte(0x1C, 0x00); // ACCEL_CONFIG
  }

  void update(float dt) {
    readRawData();
    convertToPhysical();
    updateAnglesKalman(dt);
  }

  float getRoll() const  { return _roll;  }
  float getPitch() const { return _pitch; }

private:
  static constexpr float ACCEL_SENS = 16384.0f; // LSB/g
  static constexpr float GYRO_SENS  = 131.0f;   // LSB/(deg/s)

  uint8_t _addr;

  int16_t _rawAx, _rawAy, _rawAz;
  int16_t _rawGx, _rawGy, _rawGz;

  float _ax, _ay, _az;
  float _gx, _gy, _gz;

  float _roll;
  float _pitch;

  Kalman1D _kalRoll;
  Kalman1D _kalPitch;

  void writeByte(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
  }

  void readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)_addr, (int)length, (int)true);

    uint8_t index = 0;
    while (Wire.available() && index < length) {
      buffer[index++] = Wire.read();
    }
  }

  void readRawData() {
    uint8_t buffer[14];
    readBytes(0x3B, buffer, 14); // ACCEL_XOUT_H

    _rawAx = ((int16_t)buffer[0] << 8) | buffer[1];
    _rawAy = ((int16_t)buffer[2] << 8) | buffer[3];
    _rawAz = ((int16_t)buffer[4] << 8) | buffer[5];

    _rawGx = ((int16_t)buffer[8] << 8) | buffer[9];
    _rawGy = ((int16_t)buffer[10] << 8) | buffer[11];
    _rawGz = ((int16_t)buffer[12] << 8) | buffer[13];
  }

  void convertToPhysical() {
    _ax = (float)_rawAx / ACCEL_SENS;
    _ay = (float)_rawAy / ACCEL_SENS;
    _az = (float)_rawAz / ACCEL_SENS;

    _gx = (float)_rawGx / GYRO_SENS; // deg/s
    _gy = (float)_rawGy / GYRO_SENS;
    _gz = (float)_rawGz / GYRO_SENS;
  }

  void updateAnglesKalman(float dt) {
    // Sudut dari accelerometer (deg) — sesuaikan jika orientasi fisik beda
    float rollAcc  = atan2f(_ay, _az) * 180.0f / PI;
    float pitchAcc = atan2f(-_ax, sqrtf(_ay * _ay + _az * _az)) * 180.0f / PI;

    // Update Kalman: prediksi dengan gyro, koreksi dengan accel
    _roll  = _kalRoll.update(_gx, rollAcc, dt);
    _pitch = _kalPitch.update(_gy, pitchAcc, dt);
  }
};

// ================== KELAS BMP280 ==================
class BMP280 {
public:
  BMP280(uint8_t address = 0x76)
    : _addr(address),
      _t_fine(0) {}

  bool begin() {
    uint8_t id = read8(0xD0);
    Serial.print("BMP280 ID: 0x");
    Serial.println(id, HEX);

    // terima juga 0x60 (BME280)
    if (id != 0x58 && id != 0x60) {
      Serial.println("BMP280/BME280 NOT FOUND (cek alamat 0x76 / 0x77)");
      return false;
    }

    readCalibration();

    // CTRL_MEAS: TEMP x1, PRESS x1, MODE normal
    write8(0xF4, 0b00100111);
    // CONFIG: filter off, standby default
    write8(0xF5, 0b00000000);

    return true;
  }

  // Altitude absolut (meter)
  float readAltitude() {
    int32_t rawTemp, rawPres;
    readRaw(rawTemp, rawPres);

    float tempC  = compensateTemp(rawTemp);
    float presPa = compensatePressure(rawPres);

    (void)tempC; // kalau belum dipakai

    return pressureToAltitude(presPa);
  }

private:
  uint8_t _addr;

  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
  int16_t  dig_P6, dig_P7, dig_P8, dig_P9;

  int32_t  _t_fine;

  void write8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    if (!Wire.available()) return 0;
    return Wire.read();
  }

  uint16_t read16LE(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)2);

    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    return ((uint16_t)hi << 8) | lo;
  }

  int16_t readS16LE(uint8_t reg) {
    return (int16_t)read16LE(reg);
  }

  void readCalibration() {
    dig_T1 = read16LE(0x88);
    dig_T2 = readS16LE(0x8A);
    dig_T3 = readS16LE(0x8C);

    dig_P1 = read16LE(0x8E);
    dig_P2 = readS16LE(0x90);
    dig_P3 = readS16LE(0x92);
    dig_P4 = readS16LE(0x94);
    dig_P5 = readS16LE(0x96);
    dig_P6 = readS16LE(0x98);
    dig_P7 = readS16LE(0x9A);
    dig_P8 = readS16LE(0x9C);
    dig_P9 = readS16LE(0x9E);
  }

  void readRaw(int32_t &rawTemp, int32_t &rawPres) {
    Wire.beginTransmission(_addr);
    Wire.write(0xF7);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)6);

    uint32_t pmsb = Wire.read();
    uint32_t plsb = Wire.read();
    uint32_t pxlsb = Wire.read();
    uint32_t tmsb = Wire.read();
    uint32_t tlsb = Wire.read();
    uint32_t txlsb = Wire.read();

    rawPres = (int32_t)((pmsb << 12) | (plsb << 4) | (pxlsb >> 4));
    rawTemp = (int32_t)((tmsb << 12) | (tlsb << 4) | (txlsb >> 4));
  }

  float compensateTemp(int32_t adc_T) {
    int32_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
            ((int32_t)dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
             ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    _t_fine = var1 + var2;

    float T = (_t_fine * 5 + 128) >> 8;
    return T / 100.0f;
  }

  float compensatePressure(int32_t adc_P) {
    int64_t var1, var2, p;

    var1 = ((int64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);

    var1 = (((((int64_t)1) << 47) + var1)) *
           ((int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);

    return (float)p / 256.0f; // Pa
  }

  float pressureToAltitude(float pressurePa) {
    return 44330.0f * (1.0f - pow(pressurePa / 101325.0f, 0.1903f));
  }
};

// ================== KELAS FLIGHT SYSTEM ==================
class FlightSystem {
public:
  FlightSystem()
    : _imu(0x68),
      _baro(0x76),
      _lastLoopMicros(0),
      _altitudeBaseline(0.0f),
      _baselineSet(false) {}

  void begin() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    Serial2.begin(57600, SERIAL_8N1, TELEMETRY_RX, TELEMETRY_TX);

    _imu.begin();

    if (!_baro.begin()) {
      Serial.println("BMP280 gagal diinisialisasi, cek wiring/alamat!");
    } else {
      // Set baseline altitude dengan averaging
      delay(500);
      const int N = 50;
      float sum = 0.0f;
      for (int i = 0; i < N; i++) {
        sum += _baro.readAltitude();
        delay(20);
      }
      _altitudeBaseline = sum / N;
      _baselineSet = true;

      Serial.print("Baseline altitude set to: ");
      Serial.print(_altitudeBaseline, 2);
      Serial.println(" m");
    }

    Serial.println("System ready.");
    _lastLoopMicros = micros();
  }

  void update() {
    unsigned long now = micros();
    unsigned long elapsed = now - _lastLoopMicros;

    if (elapsed >= LOOP_INTERVAL_US) {
      _lastLoopMicros = now;
      float dt = elapsed / 1000000.0f;

      // Update IMU (Kalman roll & pitch)
      _imu.update(dt);

      // Hitung altitude relatif
      float altRel = 0.0f;
      if (_baselineSet) {
        float altAbs = _baro.readAltitude();
        altRel = altAbs - _altitudeBaseline;
      }

      float roll  = _imu.getRoll();
      float pitch = _imu.getPitch();

      sendTelemetry(roll, pitch, altRel);
      checkRollAlert(roll);
    }
  }

private:
  MPU6050   _imu;
  BMP280    _baro;

  unsigned long _lastLoopMicros;
  float         _altitudeBaseline;
  bool          _baselineSet;

  void sendTelemetry(float roll, float pitch, float altitudeRelative) {
    // Telemetri ke modul
    Serial2.print("ROLL:");
    Serial2.print(roll, 2);
    Serial2.print(" deg, PITCH:");
    Serial2.print(pitch, 2);
    Serial2.print(" deg, ALT:");
    Serial2.print(altitudeRelative, 2);
    Serial2.println(" m");

    // Debug USB
    Serial.print("ROLL: ");
    Serial.print(roll, 2);
    Serial.print(" deg | PITCH: ");
    Serial.print(pitch, 2);
    Serial.print(" deg | ALT_REL: ");
    Serial.print(altitudeRelative, 2);
    Serial.println(" m");
  }

  void checkRollAlert(float roll) {
    if (fabsf(roll) > 40.0f) {
      Serial2.print("ALERT: ROLL LIMIT EXCEEDED! ROLL=");
      Serial2.print(roll, 2);
      Serial2.println(" deg");

      Serial.print("ALERT: ROLL LIMIT EXCEEDED! ROLL=");
      Serial.print(roll, 2);
      Serial.println(" deg");
    }
  }
};

// ================== GLOBAL OBJECT ==================
FlightSystem flightSystem;

// ================== ARDUINO SETUP/LOOP ==================
void setup() {
  flightSystem.begin();
}

void loop() {
  flightSystem.update();
}
