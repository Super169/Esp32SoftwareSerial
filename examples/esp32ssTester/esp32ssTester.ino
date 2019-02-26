/*
  Esp32SoftwareSerial Tester
  by Super169@2019

  Operation:
  
  1) Software serial communizaiton
     - Data received form hw serial: send to hw serial for display, and send to software serial
     - Data received from sw serial: send to hw serial for display

  2) Echo reply:
     - Data received from hw serial: ignore
     - Data received from sw serial: send to hw serial for display, and send back to software serial with time

  It seems print & printf are using difference queue, so always use printf to make sure all display in sequence
*/

#include "ESP.H"
#include <SoftwareSerial.h>

struct {
    uint8_t rx_pin          = 13;
    uint8_t tx_pin          = 12;
    unsigned long baud      = 115200;
    bool inverse_logic      = false;
    uint16_t buffer_size    = 64;
} ssConfig;

SoftwareSerial swSer(ssConfig.rx_pin , ssConfig.tx_pin, ssConfig.inverse_logic, ssConfig.buffer_size);
int mode = 1;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1, 4, 2);     //Baud rate, parity mode, RX, TX
  Serial2.begin(115200,SERIAL_8N1, 16, 17);
  swSer.begin(ssConfig.baud);
  while (Serial.available()) Serial.read();

  Serial.printf("\n\nESP32SoftwareSerial Tester\n\n");

  String fx = "";
  unsigned long lastMsgMs = 0;
  mode = 0;
  while (!mode) {
    while (!Serial.available()) {
      if (!lastMsgMs || ((millis() - lastMsgMs) > 5000)) {
        lastMsgMs = millis();
        Serial.printf("\nPlease enter (1 or 2):\n  1 - Software serial communization\n  2 - Echo reply\n");
      }
    }
    char ch = Serial.read();
    switch (ch) {
      case '1':
        mode = 1;
        fx = "software serial communization";
        break;

      case '2':
        mode = 2;
        fx = "echo reply";
        break;

      default:
        lastMsgMs = 0;
        break;
    }
    while (Serial.available()) Serial.read();
  }
  Serial.printf("\n\nStart %s\n  Rx: GPIO-%d\n  Tx: GPIO-%d\n  Baud : %ld\n\n", fx.c_str(), ssConfig.rx_pin, ssConfig.tx_pin, ssConfig.baud);
}

void loop() {
  switch (mode)
  {
    case 1:
      SSCommunization();
      break;
    case 2:
      EchoReply();
      break;
  }
}

void SSCommunization() {
  byte buffer[100];
  if (Serial.available()) {
    String data = "";
    unsigned long ms = millis();
    while (Serial.available()) {
      char c = Serial.read();
      data += String(c);
      if (!Serial.available()) delay(1);  // add 1ms delay to make sure completed
    }
    Serial.printf("%08ld >> %s\n", ms, data.c_str());
    swSer.printf("%s\n", data.c_str());
  }

  if (swSer.available()) {
    Serial.printf("%08ld << ", millis());
    while (swSer.available()) {
      char c = swSer.read();
      Serial.printf("%c", c);
      if (!swSer.available()) delay(1);
    }
    Serial.printf("\n");    
  }
}

void EchoReply() {
  if (swSer.available()) {
    unsigned long ms = millis();
    String data = "";
    while (swSer.available()) {
      char c = swSer.read();
      data = data + String(c);
      if (!swSer.available()) delay(1);
    }
    Serial.printf("%08ld << %s\n", ms, data.c_str());
    swSer.printf("%08ld << %s\n", ms, data.c_str());
  }
}