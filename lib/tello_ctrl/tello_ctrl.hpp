/* EcoDrone: Autonomous Environmental Monitoring
 * Header file for controlling the DJI Tello, contains the TelloState and TelloControl classes
 * Author: Brandon Lee, brandon.kf.lee@gmail.com
*/ 

#ifndef TELLO_CTRL_HPP
#define TELLO_CTRL_HPP

#include <Arduino.h>
#include <WiFiUdp.h>

/* Class for storing all the various state values as reported by Tello */
class TelloState{
    public: 
        TelloState(){
           num_vals = 16;
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

/* Class to faciitate the movement controls of the Tello */
class TelloControl{
    public: 
        const char* ssid = "TELLO-F1AFF9";
        const char* ip = "192.168.10.1";
        const int control_port = 8889; /* Port to send commands (control, set, read) */
        const int state_port = 8890; /* Port to recieve Tello state */

        WiFiUDP control; /* UDP port to send control signals through */
        WiFiUDP state_server; /* UDP port to recieve state updates from Tello*/
        TelloState state; /* Class for storing Tello state */

        /* Connection Methods */
        void bindPorts(IPAddress localIP);

        /* Movement Methods */
        String send_cmd_sync(const char* cmd);

        /* State Value Methods */
        void update_state_values(float val[16]);

};

#endif // TELLO_CTRL_HPP