/* EcoDrone: Autonomous Environmental Monitoring
 * Header file for creating a Bluetooth Low Energy (BLE) server to facilitate wirelesss sensor data transfer
 * Author: Brandon Lee, brandon.kf.lee@gmail.com
 * Code derived from Neil Kolban/Arduino-ESP32's BLE Notify example (https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/Notify/Notify.ino)
 */ 

#ifndef BLE_COMMS_HPP
#define BLE_COMMS_HPP

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// Drone UUID info
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

/* FreeRTOS task handle to send drone connection status updates to */
extern TaskHandle_t drone_ctrl_t;

/* Create callbacks to notify server when device connects/disconnects */
class EcoDroneBLECallbacks: public BLEServerCallbacks{
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);
};

void initBLE(String name);
boolean writeData(String msg);

#endif //BLE_COMMS_HPP