# Esp32SoftwareSerial 

Implementation of Arduino software serial library for ESP32
Support both one wire and two wire serial communization up to 115200bps

Based on the following softserial libraries, 
- https://github.com/paulvha/ESP32 (up to 56K, with problem in sending data)
- https://github.com/jdollar/espsoftwareserial (with problem in receiving data)
- https://github.com/plerup/espsoftwareserial (not work in esp32)

This project is originally for the migration of my [RobotControl] project, which need to connect multiple device via serial connection up to 115200 bps.

The initial version only fixed the connection issue and provide two wire serial connection up to 115200 bps.  Single wire connection will be added soon.

### Note:
As EspClass::getCycleCount() will be called within the interrupt, it's highly recommended to add IRAM_ATTR for this method to make sure the object is loaded into RAM instead of flash.  So you have to modify the ESP.cpp in  .platformio\packages\framework-arduinoespressif32\cores\esp32 folder.
(refer to https://github.com/paulvha/ESP32/tree/master/softserial)

[RobotControl]: <https://github.com/Super169/RobotControl>