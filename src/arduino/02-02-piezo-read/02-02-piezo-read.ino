

void setup() {
  // Enable logging through the serial port.
  Serial.begin(115200);

}


void loop() {
  int pin = 19;
  pinMode(pin, INPUT);
  Serial.println(analogRead(pin));
  delay(50);
}
