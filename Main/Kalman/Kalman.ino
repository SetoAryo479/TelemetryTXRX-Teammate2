#include "Kalman.h"

Kalman kalmanX;

void setup() {
  Serial.begin(115200);

  kalmanX.setAngle(0.0);  // set awal
}

void loop() {
  float angle = 30.0;   // contoh input accelerometer
  float rate = 1.5;     // contoh input gyroscope
  float dt = 0.01;      // delta time 10 ms

  float filtered = kalmanX.getAngle(angle, rate, dt);

  Serial.println(filtered);
  delay(10);
}
