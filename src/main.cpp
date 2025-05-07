/* EcoDrone: Autonomous Environmental Monitoring
 * Main implementation file for flying the drone & logging sensor data
 * Author: Brandon Lee, brandon.kf.lee@gmail.com
 * Code partially derived from Sensirion AG (SCD4x)
 */ 

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_NeoPixel.h>
#include <SensirionI2CScd4x.h>

#include "littlefs_io.hpp"
#include "tello_ctrl.hpp"
#include "ble_comms.hpp"

#define SEALEVELPRESSURE_HPA 1013.25

TelloControl tello;

SensirionI2CScd4x scd4x;
Adafruit_BMP3XX bmp;

TaskHandle_t sensor_read_t;
TaskHandle_t update_state_t;
TaskHandle_t drone_ctrl_t;

const char* file_name = "/data1.csv"; /* TODO: hard coded for now, fine a way to change the name every time the program is run */

/* Helper functions --------------------------------------------------------------------------------------------------------- */

/* Initialise connection from ESP32 to Tello */
void init_connection() {
   int connected;
   //Initialise in station mode, disconnect from any previous connections, and begin a connection to the specified Tello's ssid 
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    WiFi.begin(tello.ssid);
    Serial.printf("Connecting to %s ..", tello.ssid);
    while ((connected = WiFi.status()) != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }

    //Bind to Tello control & state port
    tello.bindPorts(WiFi.localIP());
    Serial.printf("%s connected.\n", tello.ssid);
}

/* Read stored sensor data from flash and send it to a device over Bluetooth Low Energy (BLE)
   A device will recieve that message, decode it, and write it out to a .csv file. 
   Returns -1 if flash is empty/cannot be read 
   Returns -2 if the BLE device has disconnected 
*/
int sendDataOverBLE(){
    /* Initialise and turn on built-in Neopixel to blue*/
    Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
    pixels.begin();
    pixels.setBrightness(50);
    pixels.fill(pixels.Color(0, 0, 255));  // R, G, B (0-255)
    pixels.show();

    /* Initialise BLE server */
    initBLE("EcoDrone_Data");

    /* Wait for drone connection task signal from ble_comms before reading from flash and writing data to BLE characteristic */
    Serial.println("Waiting for BLE connection...");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("BLE client connected. Proceeding to write...");
    
    String msg = readFile(LittleFS, file_name);
    if(msg.isEmpty()){
        Serial.print("Unable to read from flash.");
        return -1;
    }
    if(!writeData(msg)){
        Serial.print("Unable to write over BLE.");
        return -2;
    }
    Serial.println("Data successfully written.");
    pixels.fill(pixels.Color(0, 0, 0));
    pixels.show();
    
    return 1;
}
/* End helper functions --------------------------------------------------------------------------------------------------------- */

/* Tasks------------------------------------------------------------------------------------------------------------------------- */

/* Task to read Tello's state and measurements from all external sensors */
void sensor_read(void* params){
    uint16_t error;
    char errorMessage[256];
    
    writeFile(LittleFS, file_name, "Motor Time (s),Battery (%),Absolute Height (Tello TOF) (cm),CO2 (ppm),Temperature (SCD4x)(C),Temperature (BMP3xx)(C),Relative Humidity (%),Pressure (hPa),Approx. Altitude (m)");
    Serial.printf("sensor_read running on core %d\n", xPortGetCoreID());

    while(1){
        uint16_t co2;
        float scd_temp, humd, alt;
        double bmp_temp, pres;

        /* Read SCD4x measurements */
        error = scd4x.readMeasurement(co2, scd_temp, humd);
        if(error){
            /* Print out error message unless it is "NotEnoughDataError". We are polling data every second, but the SCD4x isn't ready until 5 seconds, so ignore those messages.
               Grab lower byte since NotEnoughDataError is a low level error (see SensirionErrors.cpp) */
            if ((error & 0x00FF) != NotEnoughDataError){
                //Serial.print("SCD4x: Error trying to execute readMeasurement(): ");
                errorToString(error, errorMessage, 256);
                //Serial.println(errorMessage);            
            }
            else{
                /* Zero out variables to write into file */
                /* TODO: set to -1, and if it is -1, don't put it in the "line" variable? */
                co2 = 0;
                scd_temp = 0.0;
                humd = 0;
            }
        }
        else if (co2 == 0){
            //Serial.println("SCD4x: Invalid sample detected, skipping.");
        }
        else{
            //Serial.printf("SCD4x: CO2: %d ppm, Temperature: %.2f C, Humidity: %.2f%%\n", co2, scd_temp, humd);
        }

        if(!bmp.performReading()) {
            //Serial.println("BMP3xx: Failed to perform reading.");
        }else{
            bmp_temp = bmp.temperature;
            pres = (bmp.pressure / 100.0);
            alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
            //Serial.printf("BMP3xx: Temperature: %.2f C, Pressure: %.2f hPa, Approx. Altitude: %.2f m\n", bmp_temp, pres, alt);
        }

        // TODO: Add current time as known by ESP32, relative height as reported by Tello
        String line = "\n" + String(tello.state.time) + "," + String(tello.state.bat) + "," + String(tello.state.tof) + "," + String(co2) + "," + String(scd_temp, 2) + "," + String(bmp_temp, 2) + "," + String(humd, 2) + "," + String(pres, 2) + "," + String(alt, 2);
        appendFile(LittleFS, file_name, line.c_str());
        
        //Serial.printf("Tello Battery: %d\n", tello_state.bat);
        //TODO: use neopixel to flash battery life?
        //Serial.println(line);

        delay(1000);
    }
    vTaskDelete(NULL);
}

/* Task to continiously update tello_state every 10ms in the background */
void update_state(void* params){
    while(1){
        /* Grab the first packet that comes in */
        int packetSize = tello.state_server.parsePacket();
        if (packetSize) {
            String state;
            /* Parse data into state */
            while (tello.state_server.available()) {
                char c = tello.state_server.read();
                if(c == '\n' || c == '\r'|| c == '\0'){
                    continue;
                }
                state += c;
            }
          
            /* Parse the state string
             * Format: "pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy%d;vgz:%d;templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%.2f;time:%d;agx:%.2f;agy:%.2f;agz:%.2f;\r\n"
             * The code loops through all 16 states, shifting the substring start index along as it 
             * hits each data point between the colon and semicolon
             */
            int start_ind = 0;
            int end_ind = 0;
            float vals[tello.state.num_vals];
            for(int i = 0; i < tello.state.num_vals; ++i){
                start_ind = state.indexOf(':', start_ind) + 1;
                end_ind = state.indexOf(';', start_ind);
                vals[i] = state.substring(start_ind, end_ind).toFloat();
            }
            tello.update_state_values(vals);
        }
        delay(10);
    }
    vTaskDelete(NULL);
}

/* Task to control drone movements */
void drone_ctrl(void* params){
    Serial.printf("drone_ctrl running on core %d\n", xPortGetCoreID());

    /* Blink red LED 3 times before takeoff */
    for(int i = 0; i < 2; ++i){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }

    delay(500);
    /* Start sending movement data to the drone */
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("Resp: " + tello.send_cmd_sync("takeoff"));
    Serial.println("Resp: " + tello.send_cmd_sync("up 75"));
    delay(2000);

    // Serial.println("Resp: " + tello.send_cmd_sync("forward 50"));
    // Serial.println("Resp: " + tello.send_cmd_sync("back 50"));

    Serial.println("Resp: " + tello.send_cmd_sync("land"));

    digitalWrite(LED_BUILTIN, LOW);

    /* Send data to receiving device */
    sendDataOverBLE();

    vTaskDelete(NULL);
}

/* End tasks--------------------------------------------------------------------------------------------------------------------- */

/* Begin setup  ------------------------------------------------------------------------------------------------------------------*/

void setup() {
    Serial.begin(115200);   
    // while (!Serial){
    //     delay(10);
    // }

    pinMode(LED_BUILTIN, OUTPUT);

    /* Initialise Sensirion SCD4x */
    Wire.begin();
    scd4x.begin(Wire);
    uint16_t error;
    char errorMessage[256];

    /* Stop potentially previously started measurement */
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("SCD4x: Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    /* Start Measurement */
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    /* Initialise Bosch BMP3xx 
     * hardware I2C mode, can pass in address & alt Wire */
    if (!bmp.begin_I2C()) {   
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        return;
    }

    /* Set up oversampling and filter initialization */
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);


    /* Initialise file system & file to write into */
    /* TODO: Using LittleFS library, but partition label is ffat, strange */
    /* TODO: move using LittleFS.begin into littlefs_io library, possibly make it a class */
    if (!LittleFS.begin(true, "/littlefs", 10, "ffat")){
        Serial.println("An Error has occurred while mounting LittleFS");
        return;
    }

    /* Initialise connection to Tello, enable SDK mode */
    //TODO: Split off into its own function?
    init_connection();
    String resp = tello.send_cmd_sync("command");
    if(!resp.equalsIgnoreCase("ok")){
        Serial.println("Error enabling SDK mode.");
    }
    else{
        Serial.println("Successfully entered SDK mode.");
    }

    /* Create perpetual sensor reading & flight path task*/
    xTaskCreatePinnedToCore(sensor_read, "sensor_read", 10000, NULL, 4, &sensor_read_t, 0);
    xTaskCreatePinnedToCore(update_state, "update_state", 10000, NULL, 2, &update_state_t, 0);
    xTaskCreatePinnedToCore(drone_ctrl, "drone_ctrl", 10000, NULL, 8, &drone_ctrl_t, 1);
}

void loop(){
    if(Serial){
        readFile(LittleFS, file_name);
        //Serial.printf("\n\nString data: %s\n\n", data);
        delay(5000);
    }
}