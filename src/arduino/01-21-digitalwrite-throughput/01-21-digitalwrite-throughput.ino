

void setup() { Serial.begin(115200); }

void loop() {
  {
    uint32_t start = micros();
    for (int i = 0; i < 1000; i++) {
      digitalWrite(12, HIGH);
    }
    uint32_t end = micros();
    // Display the cycle frequency.
    Serial.print("Write throughput frequency: ");
    Serial.print(1000.0 / (end - start));
    Serial.println("MHz");
  }
  {
    uint32_t start = micros();
    for (int i = 0; i < 1000; i++) {
      digitalRead(12);
    }
    uint32_t end = micros();
    // Display the cycle frequency.
    Serial.print("Read throughput frequency: ");
    Serial.print(1000.0 / (end - start));
    Serial.println("MHz");
  }
  {
    // Get port and mask from pin number.
    auto reg = portOutputRegister(digitalPinToPort(12));
    auto mask = digitalPinToBitMask(12);
    uint32_t start = micros();
    for (int i = 0; i < 1000000; i++) {
      *reg |= mask;
      *reg &= ~mask;
      *reg |= mask;
      *reg &= ~mask;      
    }
    uint32_t end = micros();
    // Display the cycle frequency.
    Serial.print("Write pad throughput frequency: ");
    Serial.print(1000000.0 / (end - start));
    Serial.println("MHz");
  }

  delay(1000);
}
