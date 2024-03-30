// From https://github.com/adafruit/Adafruit_TouchScreen/blob/master/examples/touchscreendemo/touchscreendemo.ino

// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// This demo code returns raw readings, public domain

#include <stdint.h>
#include "TouchScreen.h"

#define XM A2  // must be an analog pin, use "An" notation!
#define YP A3  // must be an analog pin, use "An" notation!
#define XP 24   // can be a digital pin
#define YM 23  // can be a digital pin

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

void setup(void) {
  Serial.begin(9600);
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}


void call_lib() {
  // a point object holds x y and z coordinates
  TSPoint p = ts.getPoint();
  
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if (p.z > ts.pressureThreshhold) {
    Serial.print("X = "); Serial.print(p.x);
    Serial.print("\tY = "); Serial.print(p.y);
    Serial.print("\tPressure = "); Serial.println(p.z);
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  }
}

void loop(void) {
  // pinMode(A0, INPUT);
  // uint32_t r = analogRead(A0);
  // Serial.print("A0: ");
  // Serial.println(r);
  // delay(100);
  call_lib();
  //call_debug_x();
  //call_debug_y();
  //Serial.println();
  //delay(200);
}

void call_debug_y(){
  // Read y
  pinMode(XP, INPUT);
  pinMode(XM, INPUT);
  pinMode(YP, OUTPUT);
  pinMode(YM, OUTPUT);

  digitalWrite(YP, HIGH);
  digitalWrite(YM, LOW);

  delay(20);
  uint32_t xp_s = analogRead(XP);
  uint32_t xm_s = analogRead(XM);

  // Print measures
  Serial.print("\tXP: ");
  Serial.print(xp_s);
  Serial.print("\tXM: ");
  Serial.print(xm_s);

  //Serial.println();
}

void call_debug_x(){

  pinMode(YP, INPUT);
  pinMode(YM, INPUT);
  pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);

  digitalWrite(XP, HIGH);
  digitalWrite(XM, LOW);

  delay(20);
  uint32_t yp_s = analogRead(YP);
  uint32_t ym_s = analogRead(YM);

  Serial.print("\tYP: ");
  Serial.print(yp_s);
  Serial.print("\tYM: ");
  Serial.print(ym_s);   
  
}