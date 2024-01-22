// the setup function runs once when you press reset or power the board
void setup() {
  // Enable logging through the serial port.
  Serial.begin(115200);
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}

#define PROFILE(x)                    \
  {                                   \
    DWT->CYCCNT = 0;                  \
    x;                                \
    const uint32_t end = DWT->CYCCNT; \
    Serial.print(#x);                 \
    Serial.print(" took ");           \
    Serial.print(end);                \
    Serial.println(" cycles");        \
  }

// the loop function runs over and over again forever
void loop() {
  auto port = digitalPinToPort(13);
  auto &reg = port->OUTSET.reg;
  auto ioreg = portOutputRegister(digitalPinToPort(12));
  PROFILE({});
  PROFILE(*ioreg |= 0x800000);

  PROFILE(REG_PORT_OUTSET0 = 0x800000);
  PROFILE(port->OUTSET.reg = 0x800000);
  PROFILE(reg = 0x800000);
  PROFILE(REG_PORT_OUTSET0 = 1 << 23);
  PROFILE(digitalPinToPort(13)->OUTSET.reg = digitalPinToBitMask(13));
  PROFILE(digitalPinToPort(13)->OUTSET.reg = 0x800000);
  PROFILE(digitalWrite(13, HIGH));

  delay(1000);  // wait for a second
  REG_PORT_OUTCLR0 = 1 << 23;
  digitalWrite(13, LOW);  // turn the LED off by making the voltage LOW
  delay(100);             // wait for a second
}