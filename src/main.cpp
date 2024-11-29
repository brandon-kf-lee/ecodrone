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
const int state_port = 8890; /* Port to recieve Tello state */
const char* file_name = "/data1.csv"; /* TODO: hard coded for now, fine a way to change the name every time the program is run */

int connected;
WiFiUDP udp;
WiFiUDP state_server;

/* TODO: split off tello functions into its own file */
class TelloState{
    public: 
        TelloState(){
            num_vals = 16;
        }

        void update_values(float vals[16]){
            pitch = vals[0];
            roll = vals[1];
            yaw = vals[2];
            vgx = vals[3];
            vgy = vals[4];
            vgz = vals[5];
            templ = vals[6];
            temph = vals[7];
            tof = vals[8];
            h = vals[9];
            bat = vals[10];
            baro = vals[11];
            time = vals[12];
            agx = vals[13];
            agy = vals[14];
            agz = vals[15];
        }
        
        int num_vals; /* Total number of state values */
        int pitch, roll, yaw; /* Drone orientation, in degrees*/
        int vgx, vgy, vgz; /* Speed in x, y, z directions */
        int templ, temph; /* Lowest and highest temperature, in celcius */
        int tof; /* Time of flight distance sensor measurement, mounted below the drone, in cm, */
        int h; /* Relative height, in cm */
        int bat; /* Battery level, in % */
        float baro; /* Barometer measurement, in cm */
        int time; /* Time since motor on, in s */
        float agx, agy, agz; /* Acceleration in x, y, z directions */
};

TelloState tello_state;

SensirionI2CScd4x scd4x;

TaskHandle_t sensor_read_t;
TaskHandle_t update_state_t;
TaskHandle_t drone_ctrl_t;


/* Helper functions --------------------------------------------------------------------------------------------------------- */

/* Initialise connection from ESP32 to Tello */
void init_connection() {
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

    //Bind to Tello control & state port
    udp.begin(WiFi.localIP(), control_port);
    state_server.begin(state_port);
}

/* TODO: lots of error checking. reference djitellopy and their implementation for what to look out for */
/* Send a synchronous command to drone (wait for and display response) */
String send_cmd_sync(const char* cmd){
    
    Serial.printf("Sending message \"%s\"... ", cmd);

    udp.beginPacket(tello_ip, control_port);
    udp.write((const uint8_t*)cmd, strlen(cmd));
    udp.endPacket();

    while(1){
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

/* Task to read Tello's state and measurements from all external sensors */
void sensor_read(void* params){
    uint16_t error;
    char errorMessage[256];
    
    writeFile(LittleFS, file_name, "Motor Time (s),Battery (%),Absolute Height (Tello TOF) (cm),CO2 (ppm),Temperature (C),Relative Humidity (%)");
    Serial.printf("sensor_read running on core %d\n", xPortGetCoreID());

    while(1){
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

        // TODO: Add current time as known by ESP32
        String line = "\n" + String(tello_state.time)  + "," + String(tello_state.bat) + "," + String(tello_state.tof) + "," + String(co2) + "," + String(temp, 2) + "," + String(humd, 2);
        appendFile(LittleFS, file_name, line.c_str());
        Serial.println(line);

        delay(1000);
    }
    vTaskDelete(NULL);
}

/* Task to continiously update tello_state every 10ms in the background */
void update_state(void* params){
    while(1){
        /* Grab the first packet that comes in */
        int packetSize = state_server.parsePacket();
        if (packetSize) {
            String state;
            /* Parse data into state */
            while (state_server.available()) {
                char c = state_server.read();
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
            float vals[tello_state.num_vals];
            for(int i = 0; i < tello_state.num_vals; ++i){
                start_ind = state.indexOf(':', start_ind) + 1;
                end_ind = state.indexOf(';', start_ind);
                vals[i] = state.substring(start_ind, end_ind).toFloat();
            }
            tello_state.update_values(vals);
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

/* Begin setup  ------------------------------------------------------------------------------------------------------------------*/

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
    init_connection();
    String resp = send_cmd_sync("command");
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

void loop(){}