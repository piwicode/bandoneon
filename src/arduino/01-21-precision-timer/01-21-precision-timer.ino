void delayCycles(uint32_t count) {
  constexpr uint32_t experimental_bias = 16;
  const uint32_t start = DWT->CYCCNT - experimental_bias;
  while (1) {
    const uint32_t elapsed = DWT->CYCCNT - start;
    if (elapsed >= count) return;
  }
}

void setup() { Serial.begin(115200); }

volatile uint32_t systickCounter = 0;

void loop() {
  // put your main code here, to run repeatedly:
  // Enable logging through the serial port.

  // Probe DEMCR.
  Serial.print("CoreDebug->DEMCR: ");
  Serial.println(CoreDebug->DEMCR);

  // Print  CoreDebug_DEMCR_TRCENA_Msk constant.
  Serial.print("CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk: ");
  Serial.println(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk);

  // Show CoreDebug->DEMCR
  Serial.print("CoreDebug->DEMCR: ");
  Serial.println(CoreDebug->DEMCR);

  // Show SystemCoreClock.
  Serial.print("SystemCoreClock: ");
  Serial.println(SystemCoreClock);

  // Show VARIANT_MCK
  Serial.print("VARIANT_MCK: ");
  Serial.println(VARIANT_MCK);

  const uint32_t occurences = 1000;
  const uint32_t occ_in_cycles = SystemCoreClock / occurences;
  uint32_t start = micros();
  for (int i = 0; i < occurences; i++) {
    delayCycles(occ_in_cycles);
  }
  uint32_t end = micros();

  Serial.print("Micros: ");
  Serial.println(end - start);

  delayMicroseconds(1000000);
}
