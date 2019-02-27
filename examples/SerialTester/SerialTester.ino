/*
  Serial Tester (for both hardware and software serial)
  by Super169@2019

  Generic tester for both hardware and software serial in EPS32.

  It seems print & printf are using difference queue, so always use printf to make sure all display in sequence
*/

#include "ESP.H"
#include <SoftwareSerial.h>

#define MSG_REPEAT_MS 5000

struct {
    uint8_t serialType      = 0;
    uint8_t rx_pin          = 0;
    uint8_t tx_pin          = 0;
    unsigned long baud      = 115200;
    bool inverse_logic      = false;
    uint16_t buffer_size    = 256;
    bool one_wire           = false;
    uint8_t mode            = 0;
} busConfig;

/*
  Default pin assignment:
    |  Serial1   |  Rx:  4  |  Tx:  2  |
    |  Serial2   |  Rx: 16  |  Tx: 17  |
    |  Software  |  Rx: 13  |  Tx: 12  |
*/

SoftwareSerial *swSer;
Stream *port;

String port_name, fx_name;

void setup() {
  Serial.begin(115200);
  ClearSerialBuffer();

  Serial.printf("\n\nSerial Tester\n\n");
  while (!GetConfig()) delay(1);

  Serial.printf("\n\n\n%s on %s\n", fx_name.c_str(), port_name.c_str());
  Serial.printf("  Rx: GPIO-%d\n  Tx: GPIO-%d\n  Baud : %ld\n\n", busConfig.rx_pin, busConfig.tx_pin, busConfig.baud);

  switch (busConfig.serialType) {
    case 1:
      Serial1.begin(115200,SERIAL_8N1, busConfig.rx_pin , busConfig.tx_pin);
      port = &Serial1;
      break;

    case 2:
      Serial2.begin(115200,SERIAL_8N1, busConfig.rx_pin , busConfig.tx_pin);
      port = &Serial2;
      break;

    default:
      swSer = new SoftwareSerial(busConfig.rx_pin , busConfig.tx_pin, busConfig.inverse_logic, busConfig.buffer_size);
      swSer->begin(busConfig.baud);
      port = swSer;
      break;
  }
}


void loop() {
  switch (busConfig.mode)
  {
    case 1:
      SerialSCommunization();
      break;
    case 2:
      EchoReply();
      break;
  }
}


bool GetConfig() {
  int defRxPin, defTxPin;

  ClearSerialBuffer();
  char sType = waitForChar("Select serial port {Software Serial}: \n  1 - Serial1, 2 - Serial2, S - Software Serial\n",
                             "12Ss ");
  Serial.printf("\n");
  switch (sType) {
    case '1':
      port_name = "Hardware Serial1";
      busConfig.serialType = 1;
      defRxPin = 4;
      defTxPin = 2;
      break;
    case '2':
      port_name = "Hardware Serial2";
      busConfig.serialType = 2;
      defRxPin = 16;
      defTxPin = 17;
      break;
    case 'S':
    case 's':
    case ' ':
    default:      
      defRxPin = 13;
      defTxPin = 12;
      port_name = "Software Serial";
      busConfig.serialType = 255;
      break;
  }
  Serial.printf("%s will be used", port_name.c_str());
  String msg;

  msg = "\n  Rx GPIO Pin {" + String(defRxPin) + "}: ";
  busConfig.rx_pin = waitForNumber(msg, 1, 35, defRxPin);
  Serial.printf("%d", busConfig.rx_pin);

  msg = "\n  Tx GPIO Pin {" + String(defTxPin) + "}: ";
  busConfig.tx_pin = waitForNumber(msg, 1, 35, defTxPin);
  Serial.printf("%d", busConfig.tx_pin);

  busConfig.one_wire = (busConfig.rx_pin == busConfig.tx_pin);
  if (busConfig.one_wire && (busConfig.serialType != 255)) {
    Serial.printf("\n\n### One wire serial connection is not supported by hardware serial.\n\n");
    return false;
  }

  busConfig.baud = waitForNumber("\n  Baud {115200}: ", 9600, 115200, 115200);
  Serial.printf("%ld\n", busConfig.baud);

  char action = waitForChar("Please select function {1}:\n  1 - Serial communization\n  2 - Echo reply\n",
                             "12 ");
  switch (action) {
    case '2':
      busConfig.mode = 2;
      fx_name = "Echo reply";
      break;

    default:
      busConfig.mode = 1;
      fx_name = "Serial communization";
      break;
  }
  return true;
}

void ClearSerialBuffer() {
  while (Serial.available()) Serial.read();
}

char waitForChar(String msg, String strOptions) {
  unsigned long lastMsgMs = 0;
  bool ready = false;
  uint8_t idx = 0;
  const char *options= strOptions.c_str();
  int count = strOptions.length();
  ClearSerialBuffer();
  while (!ready) {
    while (!Serial.available()) {
      if (!lastMsgMs || ((millis() - lastMsgMs) > MSG_REPEAT_MS)) {
        lastMsgMs = millis();
        // support formatting with printf
        Serial.printf(msg.c_str());
      }
    }
    char ch = Serial.read();
    ClearSerialBuffer();
    if (!count) return 0; // wait for any input if empty options
    idx = 0;
    while (idx < count) {
      if (ch == options[idx]) {
        ready = true;
        break;
      } 
      idx++;
    }
    // For to show message again if the option is incorrect
    if (!ready) lastMsgMs = 0;
  }
  return options[idx];
}

long waitForNumber(String msg, int iMin, int iMax) {
  return waitForNumber(msg, iMin, iMax, 0, false);
}

long waitForNumber(String msg, int iMin, int iMax, long defValue) {
  return waitForNumber(msg, iMin, iMax, defValue, true);
}

long waitForNumber(String msg, int iMin, int iMax, long defValue, bool useDefault)
{
  unsigned long lastMsgMs = 0;
  bool ready = false;
  int result = 0;
  ClearSerialBuffer();
  while (!ready) {
    while (!Serial.available()) {
      if (!lastMsgMs || ((millis() - lastMsgMs) > MSG_REPEAT_MS)) {
        lastMsgMs = millis();
        // support formatting with printf
        Serial.printf(msg.c_str());
      }
    }
    bool neg = false;
    int len = 0;
    result = 0;
    ready = true;
    while (Serial.available()) {
      char ch = Serial.read();
      if ((ch== 0x0D) || (ch == 0x0A)) {
        if ((len == 0) && (useDefault)) return defValue;
        break;
      } 
      if ((ch >= '0') && (ch <= '9')) {
        result *= 10;
        result += (ch - '0');
        len++;
      } else if (ch == '-') {
        if (neg || (len > 0)) {
          ready = false;
          break;
        }
        neg = true;
        len++;
      } else if (ch == ' ') {
        if (len > 0) {
          ready = false;
          break;
        }
      } else {
        ready = false;
        break;
      }
    }
  
    ClearSerialBuffer();
    if (ready) {
      if (neg) result *= -1;
      ready = ((result >= iMin) && (result <= iMax));
      if (ready) break;
    }
    Serial.printf("### Invalid Input ###\n");
    lastMsgMs = 0;
  }
  return result;
}

void SerialSCommunization() {
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

    // One wire only available for software serial, must enableTx before send
    if (busConfig.one_wire) swSer->enableTx(true);
    port->printf("%s\n", data.c_str());
    if (busConfig.one_wire) swSer->enableTx(false);
  }

  if (port->available()) {
    Serial.printf("%08ld << ", millis());
    while (port->available()) {
      char c = port->read();
      Serial.printf("%c", c);
      if (!port->available()) delay(1);
    }
    Serial.printf("\n");    
  }
}

void EchoReply() {
  if (port->available()) {
    unsigned long ms = millis();
    String data = "";
    while (port->available()) {
      char c = port->read();
      data = data + String(c);
      if (!port->available()) delay(1);
    }
    Serial.printf("%08ld << %s\n", ms, data.c_str());
   
    // One wire only available for software serial, must enableTx before send
    if (busConfig.one_wire) swSer->enableTx(true);
    port->printf("%08ld << %s\n", ms, data.c_str());
    if (busConfig.one_wire) swSer->enableTx(false);
  }
}