#include <Wire.h>

// ========================================
//  MPU6050 REGISTER
// ========================================
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUT_H 0x3B

// ========================================
//  BMP280 REGISTER
// ========================================
#define BMP_ADDR           0x76
#define BMP280_REG_CONTROL 0xF4
#define BMP280_REG_CONFIG  0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_DIG_T1      0x88

// ----------------------------------------
// Struktur Kalibrasi BMP280
// ----------------------------------------
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// ========================================
//  KELAS KALMAN TANPA LIBRARY
// ========================================
class SimpleKalman {
public:
    float Q_angle = 0.001f;
    float Q_bias  = 0.003f;
    float R_measure = 0.03f;

    float angle = 0;
    float bias  = 0;

    float P[2][2] = {{0,0},{0,0}};

    void setAngle(float newAngle) { angle = newAngle; }

    float getAngle(float newAngle, float newRate, float dt) {
        newRate -= bias;
        angle += dt * newRate;

        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = newAngle - angle;

        angle += K[0] * y;
        bias  += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};

// ========================================
//  KELAS IMU + BARO TANPA LIBRARY
// ========================================
class IMUBaro {
public:
    // IMU raw
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;

    float gx_off, gy_off, gz_off;

    // Angle
    float rollAcc, pitchAcc;
    float rollKal, pitchKal;

    // Kalman
    SimpleKalman Kroll, Kpitch;

    // BMP280 data
    float pressure, altitude;
    float altitudeOffset;
    bool firstAltitude = true;

    unsigned long lastTime = 0;

    bool begin() {
        Wire.begin();
        initMPU6050();
        initBMP280();
        calibrateGyro();
        initKalman();
        lastTime = micros();
        return true;
    }

    // Main update
    void update() {
        float dt = (micros() - lastTime) / 1e6;
        lastTime = micros();

        readMPU6050();
        calculateAngles(dt);
        readBMP280();
    }

    // Getter
    float getRollKalman()  { return rollKal; }
    float getPitchKalman() { return pitchKal; }
    float getAltitude()    { return altitude; }

private:

    // ---------------- MPU6050 ----------------
    void initMPU6050() {
        write8(MPU_ADDR, MPU_PWR_MGMT_1, 0x00);
    }

    void readMPU6050() {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MPU_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);

        accX = Wire.read()<<8 | Wire.read();
        accY = Wire.read()<<8 | Wire.read();
        accZ = Wire.read()<<8 | Wire.read();
        gyroX = Wire.read()<<8 | Wire.read();
        gyroY = Wire.read()<<8 | Wire.read();
        gyroZ = Wire.read()<<8 | Wire.read();
    }

    void calibrateGyro() {
        long sx=0, sy=0, sz=0;
        for(int i=0;i<500;i++){
            readMPU6050();
            sx+=gyroX; sy+=gyroY; sz+=gyroZ;
            delay(3);
        }
        gx_off = sx/500.0;
        gy_off = sy/500.0;
        gz_off = sz/500.0;
    }

    void initKalman() {
        readMPU6050();
        rollAcc  = atan2(accY, accZ)*180/PI;
        pitchAcc = atan2(-accX, sqrt(accY*accY+accZ*accZ))*180/PI;

        Kroll.setAngle(rollAcc);
        Kpitch.setAngle(pitchAcc);
    }

    void calculateAngles(float dt) {
        rollAcc  = atan2(accY, accZ)*180/PI;
        pitchAcc = atan2(-accX, sqrt(accY*accY+accZ*accZ))*180/PI;

        float gx = (gyroX - gx_off)/131.0;
        float gy = (gyroY - gy_off)/131.0;

        rollKal  = Kroll.getAngle(rollAcc,  gx, dt);
        pitchKal = Kpitch.getAngle(pitchAcc, gy, dt);
    }

    // ---------------- BMP280 ----------------
    void initBMP280() {
        readBMP280Calibration();
        write8(BMP_ADDR, BMP280_REG_CONTROL, 0x27);
        write8(BMP_ADDR, BMP280_REG_CONFIG, 0xA0);
    }

    void readBMP280() {
        int32_t adc_P = read24(BMP280_REG_PRESS_MSB);

        // Compensation dari datasheet
        int32_t var1, var2;
        var1 = (((adc_P>>3) - ((int32_t)dig_P1<<1)) * dig_P2) >> 11;
        var2 = (((((adc_P>>4) - (int32_t)dig_P1) * ((adc_P>>4) - (int32_t)dig_P1)) >> 12) *
                dig_P3) >> 14;

        int32_t p = (var1 + var2);
        if (p == 0) return;
        p = ((((adc_P) - ((p >> 8))) * 3125));

        pressure = p / 100.0;

        float altitudeRaw = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));

        if (firstAltitude) {
            altitudeOffset = altitudeRaw;
            firstAltitude = false;
        }

        altitude = altitudeRaw - altitudeOffset;
    }

    void readBMP280Calibration() {
        dig_T1 = read16(BMP280_DIG_T1);
        dig_T2 = readS16(BMP280_DIG_T1+2);
        dig_T3 = readS16(BMP280_DIG_T1+4);

        dig_P1 = read16(BMP280_DIG_T1+6);
        dig_P2 = readS16(BMP280_DIG_T1+8);
        dig_P3 = readS16(BMP280_DIG_T1+10);
        dig_P4 = readS16(BMP280_DIG_T1+12);
        dig_P5 = readS16(BMP280_DIG_T1+14);
        dig_P6 = readS16(BMP280_DIG_T1+16);
        dig_P7 = readS16(BMP280_DIG_T1+18);
        dig_P8 = readS16(BMP280_DIG_T1+20);
        dig_P9 = readS16(BMP280_DIG_T1+22);
    }

    // -------------- I2C Helpers ----------------
    uint16_t read16(uint8_t reg) {
        Wire.beginTransmission(BMP_ADDR);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(BMP_ADDR, 2);
        return (Wire.read()<<8 | Wire.read());
    }

    int16_t readS16(uint8_t reg) {
        return (int16_t)read16(reg);
    }

    uint32_t read24(uint8_t reg) {
        Wire.beginTransmission(BMP_ADDR);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(BMP_ADDR, 3);
        return (Wire.read()<<16 | Wire.read()<<8 | Wire.read());
    }

    void write8(uint8_t addr, uint8_t reg, uint8_t value) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
};

// ========================================
// GLOBAL OBJECT
// ========================================
IMUBaro imu;

// ========================================
// ARDUINO SYSTEM
// ========================================
void setup() {
    Serial.begin(115200);
    imu.begin();
}

void loop() {
    imu.update();

    Serial.print("Roll: ");
    Serial.print(imu.getRollKalman());
    Serial.print(" deg |")
    Serial.print("  Pitch: ");
    Serial.print(imu.getPitchKalman());
    Serial.print(" deg |")
    Serial.print("  Altitude: ");
    Serial.print(imu.getAltitude());
    Serial.println(" m |")

    delay(20);
}
