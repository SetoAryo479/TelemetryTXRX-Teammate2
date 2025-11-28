#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

// Serial2 untuk telemetry
// RX = 16, TX = 17 (bisa diubah sesuai wiring)
HardwareSerial Telemetry(2);

float pressure, altitude;
float altitudeOffset = 0;
bool firstAltitudeRead = true;

void setup() {
  Serial.begin(115200);
  Telemetry.begin(57600, SERIAL_8N1, 16, 17);

  // Inisialisasi BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 tidak terdeteksi!");
    while (1);
  }

  // Konfigurasi sampling BMP280 (lebih stabil untuk telemetry)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_125);

  Serial.println("BMP280 siap dan telemetry aktif...");
}

void loop() {
  // Baca tekanan (hPa)
  pressure = bmp.readPressure() / 100.0;

  // Baca altitude (meter)
  float altitudeRaw = bmp.readAltitude(1013.25);   // 1013.25 = tekanan referensi permukaan laut

  // Offset altitude pada pembacaan pertama → supaya altitude = 0 pada titik awal
  if (firstAltitudeRead) {
    altitudeOffset = altitudeRaw;
    firstAltitudeRead = false;
  }
  altitude = altitudeRaw - altitudeOffset;

  // Buat string data telemetry
  String data = 
      String("ALT:") + altitude + 
      ",PRS:" + pressure;

  // Kirim ke Serial monitor
  Serial.println(data);

  // Kirim ke module telemetry via Serial2
  Telemetry.println(data);

  delay(100); // 10 Hz
}
