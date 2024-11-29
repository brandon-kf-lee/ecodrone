/* EcoDrone: Autonomous Environmental Monitoring
 * Main implementation file for flying the drone & logging sensor data
 * Author: Brandon Lee, brandon.kf.lee@gmail.com
 * Code partially derived from Sensirion AG (SCD4x)
 */ 

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SensirionI2CScd4x.h>

#include "littlefs_io.hpp"

const char* ssid = "TELLO-F1AFF9";
const char* tello_ip = "192.168.10.1";
const int control_port = 8889; /* Port to send commands (control, set, read) */
const int status_port = 8890; /* (Unused) port to recieve Tello status */
const char* file_name = "/data1.csv"; /* TODO: hard coded for now, fine a way to change the name every time the program is run */

int connected;
WiFiUDP udp;

SensirionI2CScd4x scd4x;

TaskHandle_t sensor_read_t;
TaskHandle_t drone_ctrl_t;


/* Helper functions --------------------------------------------------------------------------------------------------------- */
/* Initialise connection from ESP32 to Tello */
void initConnection() {
    //Initialise in station mode, disconnect from any previous connections, and begin a connection to the specified Tello's ssid 
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    WiFi.begin(ssid);
    Serial.printf("Connecting to %s ..", ssid);
    while ((connected = WiFi.status()) != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }
    Serial.printf("%s connected.\n", ssid);

    //Bind to Tello
    udp.begin(WiFi.localIP(), control_port);
    Serial.println(WiFi.localIP()); //192.168.10.2
}

/* TODO: lots of error checking. reference djitellopy and their implementation for what to look out for */
/* Send a synchronous command to drone (wait for and display response) */
String send_cmd_sync(const char* cmd){
    
    Serial.printf("Sending message \"%s\"... ", cmd);

    udp.beginPacket(tello_ip, control_port);
    udp.write((const uint8_t*)cmd, strlen(cmd));
    udp.endPacket();

    while (1) {
        int packetSize = udp.parsePacket();
        if (packetSize){
            String resp;
            while (udp.available()) {
                char c = udp.read();
                if(c == '\n' || c == '\r'|| c == '\0'){
                    continue;
                }
                resp += c;
            }
            return resp;
        } 
    }
}

/* End helper functions --------------------------------------------------------------------------------------------------------- */

/* Tasks------------------------------------------------------------------------------------------------------------------------- */
/* Task to read status of Tello and measurements from all external sensors */
void sensor_read(void* params){
    uint16_t error;
    char errorMessage[256];
    
    writeFile(LittleFS, file_name, "Time,Battery,Tello TOF,CO2,Temperature,Humidity");
    Serial.printf("sensor_read running on core %d\n", xPortGetCoreID());

    while(1){
        
        String tello_tof, battery, time = "";
        /* Read status from Tello */
        /* TODO: does not work after landing, blocks sensors from reading. Maybe attempt to reconnect to tello after landing (add reconnection functionality to send_cmd_sync?) */
        /* TODO: request Tello state from the ohter port since udp messages seem to be colliding*/
        Serial.println("Tello: " + (tello_tof = send_cmd_sync("tof?")));
        Serial.println("Tello: " + (battery = send_cmd_sync("battery?")) + "%");
        Serial.println("Tello: " + (time = send_cmd_sync("time?")));

        /* Read SCD4x measurements */
        uint16_t co2;
        float temp, humd;
        error = scd4x.readMeasurement(co2, temp, humd);

        if(error){
            /* Print out error message unless it is "NotEnoughDataError". We are polling data every second, but the SCD4x isn't ready until 5 seconds, so ignore those messages.
               Grab lower byte since NotEnoughDataError is a low level error (see SensirionErrors.cpp) */
            if ((error & 0x00FF) != NotEnoughDataError){
                Serial.print("SCD4x: Error trying to execute readMeasurement(): ");
                errorToString(error, errorMessage, 256);
                Serial.println(errorMessage);            
            }
            else{
                /* Zero out variables to write into file */
                co2 = 0;
                temp = 0.0;
                humd = 0;
            }
        }
        else if (co2 == 0){
            Serial.println("SCD4x: Invalid sample detected, skipping.");
        }
        else{
            //Serial.printf("SCD4x: CO2: %d, Temperature: %.2f, Humidity: %.2f\n", co2, temp, humd);
        }

        // Add current time and/or time drone spent online?
        String line = "\n" + time  + "," + battery + "," + tello_tof + "," + String(co2) + "," + String(temp, 2) + "," + String(humd, 2);

        char buf[line.length()];
        line.toCharArray(buf, line.length());

        appendFile(LittleFS, file_name, buf);
        //readFile(LittleFS, file_name);

        delay(1000);
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
    digitalWrite(LED_BUILTIN, HIGH);


    Serial.println("Resp: " + send_cmd_sync("takeoff"));
    delay(3000);
    Serial.println("Resp: " + send_cmd_sync("land"));

    digitalWrite(LED_BUILTIN, LOW);


    readFile(LittleFS, file_name);

    //TODO: Start advertising bluetooth connection, flash neopixel led blue?

    vTaskDelete(NULL);
}

/* End tasks--------------------------------------------------------------------------------------------------------------------- */

void setup() {
    Serial.begin(115200);   
    while (!Serial){
        delay(10);
    }

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

    /* Initialise file system & file to write into */
    /* TODO: Using LittleFS library, but partition label is ffat, strange */
    /* TODO: move using LittleFS.begin into littlefs_io library, possibly make it a class */
    if (!LittleFS.begin(true, "/littlefs", 10, "ffat")){
        Serial.println("An Error has occurred while mounting LittleFS");
        return;
    }

    /* Initialise connection to Tello, enable SDK mode */
    //TODO: Split off into its own function?
    initConnection();
    String resp = send_cmd_sync("command");
    if(!resp.equalsIgnoreCase("ok")){
        Serial.println("Error enabling SDK mode.");
    }

    /* Create perpetual sensor reading & flight path task*/
    xTaskCreatePinnedToCore(sensor_read, "sensor_read", 10000, NULL, 1, &sensor_read_t, 0);
    xTaskCreatePinnedToCore(drone_ctrl, "drone_ctrl", 10000, NULL, 1, &drone_ctrl_t, 1);
}

void loop() {
    /* Unused */
}