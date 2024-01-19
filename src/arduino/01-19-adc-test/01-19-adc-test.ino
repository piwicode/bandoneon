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
  while (!Serial);

  Serial.println("MCP3008 simple test.");

  // Hardware SPI (specify CS, use any available digital)
  // adc.begin(13); 

  // Software SPI.
  adc.begin(10, 12, 11, 13);
}

void loop() {
  for (int chan=0; chan<8; chan++) {
    Serial.print(adc.readADC(chan)); Serial.print("\t");
  }

  Serial.print("["); Serial.print(count); Serial.println("]");
  count++;
  
  delay(100);
}
