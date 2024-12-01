# EcoDrone: Autonomous Environmental Monitoring

## Overview
This project integrates a DJI Tello and an ESP32-S3 microcontroller with sensors into one autonomous system. Sensors include true CO2 concentration, temperature, relative humidity, and barometric pressure. Includes ability to fly drone autonomously on pre-planned flight paths and logs sensor data onto the on-board flash.

* Built in [PlatformIO](https://platformio.org/) through Visual Studio Code

## How This Project Works
The Tello drone implements an SDK that allows external devices to communicate with it and issue commands through UDP. The ESP32, which lives on top of the drone, issues those commands in the form of directions that follow a pre-programmed flight path. Simultaneously, the ESP32 is logging sensor data onto its flash storage to be retrieved at a later time. 

* The ESP32 will connect to and communicate with the Tello over WiFi/UDP, issuing directional commands.
    * ESP32 will read in its state data (battery percentage, motor time one, etc.) from Tello during runtime.
    * For commands, the Tello is the server (has an SSID to connect to), ESP32 is the client.
    * For state data, the ESP32 is the server and the Tello connects to it as a client (see SDK for more details).
* The ESP32 will be preprogrammed with flight path data to send to Tello.
* The ESP32 will record the Tello's state data and its own sensor data into a file through LittleFS in the csv format. 
* (Tentative) Continuously broadcast a BLE connection during runtime, used for reporting sensor data (not intended to be connected to until flight completion)
* (Tentative) After the drone lands, bring an external computer to connect to the ESP32 through Bluetooth LE, and transmit data from the ESP32 to the computer

## Dependencies
* Python
    * [bleak](https://github.com/hbldh/bleak): Used to connect an external device to the ESP32 through the BLE (Bluetooth Low Energy) module for communication

## Hardware Used

* [DJI Tello](https://www.ryzerobotics.com/tello)
    * Using [SDK 1.3.0.0](https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf) (other versions are incompatable with this version of the drone)
* [Adafruit ESP32-S3 Feather](https://www.adafruit.com/product/5323)
* Sensors
    * [SCD-41](https://www.adafruit.com/product/5190) - True CO2, Temperature, and Humidity Sensor
    * [BMP390](https://www.adafruit.com/product/4816) - Barometric Pressure Sensor and Altimeter
    * [VL53L4CX](https://www.adafruit.com/product/5425) - Time of Flight Distance Sensor
* Communication Protocols
    * All sensors communicate through I2C (using Adafruit's STEMMA QT)
    * The ESP32 communicates with the Tello through UDP

## Authors and Contributors
* [Brandon Lee](https://brandon-kf-lee.github.io/)
* [Dr. Tom Springer](https://www.chapman.edu/our-faculty/tom-springer), Research Advisor
* Funded through Chapman University's Undergraduate Research Grant