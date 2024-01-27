/***************************************************
Simple example of reading the MCP3008 analog input channels and printing
them all out.

Author: Carter Nelson
License: Public Domain
****************************************************/
#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

int count = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println("MCP3008 simple test.");

  // Hardware SPI (specify CS, use any available digital)
  // adc.begin(13);

  // Software SPI.git
  adc.begin(10, 12, 21, 13);
}

void loop() {
  count++;

  Serial.print("[");
  Serial.print(count);
  Serial.print("] ");
  for (int chan = 0; chan < 8; chan++) {
    Serial.print(adc.readADC(chan));
    Serial.print("\t");
  }


  // Small benchmark for read frequency.
  auto start = micros();
  auto sample_count = 100;
  for (int i = 0; i < sample_count; i++) {
    adc.readADC(0);
  }
  auto end = micros();
  // Show read frequency.
  Serial.print("Read frequency: ");
  Serial.print(1e6 * sample_count / (end - start));
  Serial.println(" sps");
  delay(100);
}
