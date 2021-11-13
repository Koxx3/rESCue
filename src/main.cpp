#include <Arduino.h>
#include <Logger.h>
#include "config.h"
#include "BleServer.h"
#include "CanBus.h"
#include "AppConfiguration.h"


HardwareSerial vesc(2);

BleServer *bleServer = new BleServer();

#if defined(CANBUS_ENABLED)
 CanBus * canbus = new CanBus();
#endif //CANBUS_ENABLED

// Declare the local logger function before it is called.
void localLogger(Logger::Level level, const char* module, const char* message);

void setup() {
  Logger::setOutputFunction(localLogger);
  Logger::setLogLevel(Logger::NOTICE);

  AppConfiguration::getInstance()->readPreferences();
  Logger::setLogLevel(AppConfiguration::getInstance()->config.logLevel);
  if(Logger::getLogLevel() != Logger::SILENT) {
      Serial.begin(921600);
  }

  vesc.begin(VESC_BAUD_RATE, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN, false);      
  delay(50);
#ifdef CANBUS_ENABLED
  // initializes the CANBUS
  canbus->init();
#endif //CANBUS_ENABLED

  // initialize the UART bridge from VESC to BLE and the BLE support for Blynk (https://blynk.io)
#ifdef CANBUS_ONLY
  bleServer->init(canbus->stream, canbus);
#else
  bleServer->init(&vesc, canbus);
#endif

  char buf[128];
  snprintf(buf, 128, " sw-version %d.%d.%d is happily running on hw-version %d.%d", 
    SOFTWARE_VERSION_MAJOR, SOFTWARE_VERSION_MINOR, SOFTWARE_VERSION_PATCH, 
    HARDWARE_VERSION_MAJOR, HARDWARE_VERSION_MINOR);
  Logger::notice("rESCue", buf);
}

void loop() {

#ifdef CANBUS_ENABLED
  canbus->loop();
#endif

  // call the VESC UART-to-Bluetooth bridge
#ifdef CANBUS_ENABLED
  bleServer->loop(&canbus->vescData);
#else 
  bleServer->loop();
#endif
}

void localLogger(Logger::Level level, const char* module, const char* message) {
  Serial.print(F("FWC: ["));
  Serial.print(Logger::asString(level));
  Serial.print(F("] "));
  if (strlen(module) > 0) {
      Serial.print(F(": "));
      Serial.print(module);
      Serial.print(" ");
  }
  Serial.println(message);
}