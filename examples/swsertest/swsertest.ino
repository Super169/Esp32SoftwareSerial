#include <SoftwareSerial.h>

// As both during TX and RX softSerial is disabling interrupts
// you can not send data from RX to TX on a same ESP32
// left the original example in just to show how to use

// RX = pin 14, TX = 12, none-invert, buffersize 256.
SoftwareSerial swSer(14, 12, false, 256);

void setup() {
  Serial.begin(115200);
  swSer.begin(115200);

  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");

}

void loop() {
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
  }

}
