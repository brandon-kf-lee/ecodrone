# EcoDrone: Autonomous Environmental Monitoring

## Overview
This project integrates a DJI Tello and an ESP32-S3 microcontroller with sensors into one autonomous system. Sensors include true CO2 concentration, temperature, relative humidity, and barometric pressure. Includes ability to fly drone autonomously on pre-planned flight paths, and currently logs sensor data onto the serial monitor.

* Built in [PlatformIO](https://platformio.org/) through Visual Studio Code

## How This Project Works
The Tello drone implements an SDK that allows external devices to communicate with it and issue commands through UDP. The ESP32, which lives on top of the drone, issues those commands in the form of directions that follow a pre-programmed flight path. Simultaneously, the ESP32 is logging sensor data to be retrieved at a later time. 

* The ESP32 will connect to and communicate with the Tello over WiFi, issuing directional commands
    * In this case, the Tello is the server (has an SSID to connect to), ESP32 is the client
    * The ESP32 will be preprogrammed with flight path data to send to Tello
    * ESP32 will read in battery data & other metrics from Tello during runtime
* (Tentative) Continuously broadcast a BLE connection during runtime, used for reporting sensor data (not intended to be connected to until flight completion)
* (Tentative) ESP32 will record its own sensor data into a file through LittleFS
* (Tentative) After the drone lands, bring an external computer to connect to the ESP32 through Bluetooth LE, and transmit data from the ESP32 to the computer

## Hardware Used

* [DJI Tello](https://www.ryzerobotics.com/tello)
    * Using [SDK 1.3.0.0](https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf) (other versions are incompatable with this version of the drone)
* [Adafruit ESP32-S3 Feather](https://www.adafruit.com/product/5323)
* Sensors
    * [SCD-41](https://www.adafruit.com/product/5190) - True CO2, Temperature, and Humidity Sensor
    * [BMP390](https://www.adafruit.com/product/4816) - Barometric Pressure Sensor and Altimeter
    * [VL53L4CX](https://www.adafruit.com/product/5425) - Time of Flight Distance Sensor

## Authors and Contributors
* Brandon Lee
* [Dr. Tom Springer](https://www.chapman.edu/our-faculty/tom-springer), Research Advisor
* Funded through Chapman University's Undergraduate Research Grant