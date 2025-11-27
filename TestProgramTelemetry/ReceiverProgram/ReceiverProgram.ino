HardwareSerial Telemetry(2);

void setup() {
  Serial.begin(115200);
  Telemetry.begin(57600, SERIAL_8N1, 16, 17);
  Serial.println("Menunggu data telemetry...");
}

void loop() {
  if (Telemetry.available()) {
    char c = Telemetry.read();
    Serial.write(c); // tampil di Serial Monitor
  }
}
