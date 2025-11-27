HardwareSerial Telemetry(2); // UART2

void setup() {
  Serial.begin(115200);
  Telemetry.begin(57600, SERIAL_8N1, 16, 17); // RX=16, TX=17 → sesuaikan wiring
  Serial.println("Mulai test Telemetry...");
}

void loop() {
  Telemetry.println("Hello from Telemetry!");
  Serial.println("Data terkirim...");
  delay(1000);
}
