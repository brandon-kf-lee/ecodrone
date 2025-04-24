/* EcoDrone: Autonomous Environmental Monitoring
 * Implementation file for creating a Bluetooth Low Energy (BLE) server to facilitate wirelesss sensor data transfer
 * Author: Brandon Lee, brandon.kf.lee@gmail.com
 * Code derived from Neil Kolban/Arduino-ESP32's BLE Notify example (https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/Notify/Notify.ino)
 */ 

#include "ble_comms.hpp"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

void EcoDroneBLECallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
    
    if(drone_ctrl_t != NULL){
      xTaskNotifyGive(drone_ctrl_t);
    }
};

void EcoDroneBLECallbacks::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
}

/* Initialise BLE server, start advertising connection  */
void initBLE(String name) {
  BLEDevice::init(name.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new EcoDroneBLECallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Start service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.println("Advertising started. Waiting for client connection...");
}

/* Write msg data to the BLE server's characteristic */
boolean writeData(String msg){
    if (deviceConnected) {
        pCharacteristic->setValue(msg.c_str());
        pCharacteristic->notify(); // Send value over BLE
        Serial.println("Notified value: \n" + msg);
    }
    return deviceConnected;
}