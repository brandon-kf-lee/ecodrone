/* EcoDrone: Autonomous Environmental Monitoring
 * Implementation file for controlling the DJI Tello
 * Author: Brandon Lee, brandon.kf.lee@gmail.com
*/ 

#include "tello_ctrl.hpp"

// Given the IP address of the ESP32, bind it to Tello control & state ports
void TelloControl::bindPorts(IPAddress localIP){
    control.begin(localIP, control_port);
    state_server.begin(state_port);
}

/* TODO: lots of error checking. reference djitellopy and their implementation for what to look out for */
/* Send a synchronous command to drone (wait for and display response) */
String TelloControl::send_cmd_sync(const char* cmd){
    
    Serial.printf("Sending message \"%s\"... ", cmd);

    control.beginPacket(ip, control_port);
    control.write((const uint8_t*)cmd, strlen(cmd));
    control.endPacket();

    while(1){
        int packetSize = control.parsePacket();
        if (packetSize){
            String resp;
            while (control.available()) {
                char c = control.read();
                if(c == '\n' || c == '\r'|| c == '\0'){
                    continue;
                }
                resp += c;
            }
            return resp;
        } 
    }
}

/* Given 16 float values from Tello, update the TelloState class's values */
void TelloControl::update_state_values(float vals[16]){
    state.pitch = vals[0];
    state.roll = vals[1];
    state.yaw = vals[2];
    state.vgx = vals[3];
    state.vgy = vals[4];
    state.vgz = vals[5];
    state.templ = vals[6];
    state.temph = vals[7];
    state.tof = vals[8];
    state.h = vals[9];
    state.bat = vals[10];
    state.baro = vals[11];
    state.time = vals[12];
    state.agx = vals[13];
    state.agy = vals[14];
    state.agz = vals[15];
}