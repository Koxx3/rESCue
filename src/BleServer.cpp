#include "BleServer.h"
#include <Logger.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"

const int BLE_PACKET_SIZE = 128;
NimBLEServer *pServer = NULL;
NimBLEService *pServiceVesc = NULL;
NimBLECharacteristic *pCharacteristicVescTx = NULL;
NimBLECharacteristic *pCharacteristicVescRx = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
Stream *vescSerial;
std::string bufferString;
int lastLoop = 0;
int lastNotification = 0;
int lastBatteryValue = 0;

void syncPreferencesWithApp();

BleServer::BleServer() : mConn(false),
                         mName(BT_NAME) {}

// NimBLEServerCallbacks::onConnect
inline void BleServer::onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
{
  char buf[128];
  snprintf(buf, 128, "Client connected: %s", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
  Logger::notice(LOG_TAG_BLESERVER, buf);
  Logger::notice(LOG_TAG_BLESERVER, "Multi-connect support: start advertising");
  deviceConnected = true;
  NimBLEDevice::startAdvertising();
};

// NimBLEServerCallbacks::onDisconnect
inline void BleServer::onDisconnect(NimBLEServer *pServer)
{
  Logger::notice(LOG_TAG_BLESERVER, "Client disconnected - start advertising");
  deviceConnected = false;
  NimBLEDevice::startAdvertising();
}

void BleServer::init(Stream *vesc, CanBus *canbus)
{
  vescSerial = vesc;

  // Create the BLE Device
  NimBLEDevice::init(BT_NAME);

  this->canbus = canbus;

  // Create the BLE Server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(this);
  auto pSecurity = new NimBLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  // Create the VESC BLE Service
  pServiceVesc = pServer->createService(VESC_SERVICE_UUID);

  // Create a BLE Characteristic for VESC TX
  pCharacteristicVescTx = pServiceVesc->createCharacteristic(
      VESC_CHARACTERISTIC_UUID_TX,
      NIMBLE_PROPERTY::NOTIFY |
          NIMBLE_PROPERTY::READ);
  // pCharacteristicVescTx->setValue("VESC TX");
  pCharacteristicVescTx->setCallbacks(this);

  // Create a BLE Characteristic for VESC RX
  pCharacteristicVescRx = pServiceVesc->createCharacteristic(
      VESC_CHARACTERISTIC_UUID_RX,
      NIMBLE_PROPERTY::READ |
          NIMBLE_PROPERTY::WRITE);
  // pCharacteristicVescRx->setValue("VESC RX");
  pCharacteristicVescRx->setCallbacks(this);

  // Start the VESC service
  pServiceVesc->start();

  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(VESC_SERVICE_UUID);
  pAdvertising->setAppearance(0x00);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter

  NimBLEDevice::startAdvertising();
  Logger::notice(LOG_TAG_BLESERVER, "waiting a client connection to notify...");
}

#ifdef CANBUS_ENABLED
void BleServer::loop(CanBus::VescData *vescData)
{
#else
void BleServer::loop()
{
#endif
  if (vescSerial->available())
  {
    int oneByte;
    while (vescSerial->available())
    {
      oneByte = vescSerial->read();
      bufferString.push_back(oneByte);
    }
    /*
    if (Logger::getLogLevel() == Logger::NOTICE)
    {
      size_t len = bufferString.length();
      char buffer[128];
      snprintf(buffer, 128, "VESC => BLE/UART : len = %d", len);
      Logger::notice(LOG_TAG_BLESERVER, buffer);
    }
    */

    if (deviceConnected)
    {
      while (bufferString.length() > 0)
      {
        if (bufferString.length() > BLE_PACKET_SIZE)
        {

          std::string shortBuffer = bufferString.substr(0, BLE_PACKET_SIZE);

          //----------------
          Serial.printf("BluetoothHandler VESC => BLE/UART : len = %d / ", bufferString.length());
          for (int i = 0; i < shortBuffer.size(); i++)
          {
            Serial.printf("%02x ", shortBuffer.at(i));
          }
          Serial.println();
          //----------------

          pCharacteristicVescTx->setValue(shortBuffer);
          bufferString = bufferString.substr(BLE_PACKET_SIZE);
        }
        else
        {

          //----------------
          Serial.printf("BluetoothHandler VESC => BLE/UART : len = %d / ", bufferString.length());
          for (int i = 0; i < bufferString.size(); i++)
          {
            Serial.printf("%02x ", bufferString.at(i));
          }
          Serial.println();
          //----------------

          pCharacteristicVescTx->setValue(bufferString);
          bufferString.clear();
        }
        pCharacteristicVescTx->notify();
        delay(10); // bluetooth stack will go into congestion, if too many packets are sent
      }
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Logger::notice(LOG_TAG_BLESERVER, "start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

// NimBLECharacteristicCallbacks::onWrite
void BleServer::onWrite(BLECharacteristic *pCharacteristic)
{
  char buffer[128];
  std::string rxValue = pCharacteristic->getValue();
  size_t len = rxValue.length();
  snprintf(buffer, 128, "BLE/UART => VESC : len = %d", len);
  Logger::notice(LOG_TAG_BLESERVER, buffer);
  if (rxValue.length() > 0)
  {
    if (pCharacteristic->getUUID().equals(pCharacteristicVescRx->getUUID()))
    {
      /*
            for (int i = 0; i < rxValue.length(); i++) {
              Serial.print(rxValue[i], DEC);
              Serial.print(" ");
            }
            Serial.println();
      */

#ifdef CANBUS_ONLY
      canbus->proxyIn(rxValue);
#else
      for (int i = 0; i < rxValue.length(); i++)
      {
        vescSerial->write(rxValue[i]);
      }
#endif
    }
  }
}

// NimBLECharacteristicCallbacks::onSubscribe
void BleServer::onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue)
{
  char buf[256];
  snprintf(buf, 256, "Client ID: %d, Address: %s, Subvalue %d, Characteristics %s ",
           desc->conn_handle, NimBLEAddress(desc->peer_ota_addr).toString().c_str(), subValue, pCharacteristic->getUUID().toString().c_str());
  Logger::notice(LOG_TAG_BLESERVER, buf);
}

// NimBLECharacteristicCallbacks::onSubscribe
void BleServer::onStatus(NimBLECharacteristic *pCharacteristic, Status status, int code)
{
  char buf[256];
  snprintf(buf, 256, "Notification/Indication status code: %d, return code: %d", status, code);
  Logger::verbose(LOG_TAG_BLESERVER, buf);
}
