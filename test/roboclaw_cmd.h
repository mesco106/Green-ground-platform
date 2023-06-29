#ifndef ROBOCLAW_CMD_H
#define ROBOCLAW_CMD_H

#define address 0x80

#include <RoboClaw.h>
#include <motion_control.h>
#include <remote_control.h>
#include <init.h>

#include "Arduino.h"
#include "SerialPort.h"
#include "Gpio.h"

RoboClaw roboclaw1(&Serial3, 10000);
RoboClaw roboclaw2(&Serial4, 10000);

extern int state;


namespace roboclawCommands{
    
    void run(const String &message);
    float convertTemperature(uint16_t rawTemperature);

    //SerialPort port(115200, roboclawCommands::run);

    void run(const String &message) {
        auto action = command(message);
        port.send(action);
        if (action == "on") {
            //led.on();

        } else if (action == "off") {
            //led.off();

        } else if (action == "hi") {
            port.send("Hi there!");

        } else if (action == "fo") {
            float speed = argument(message, 1);
            motionControl::forward(*ptr_rc1, *ptr_rc2, speed, address);
            port.send("Moving forwards at: " + String(speed));

        } else if (action == "ba") {
            float speed = argument(message, 1);
            motionControl::backward(*ptr_rc1, *ptr_rc2, speed, address);
            port.send("Moving backwards at: " + String(speed));

        } else if (action == "le") {
            float speed = argument(message, 1);
            motionControl::front(*ptr_rc1, *ptr_rc2, speed, address);
            port.send("Moving front at: " + String(speed));

        } else if (action == "ri") {
            float speed = argument(message, 1);
            motionControl::rear(*ptr_rc1, *ptr_rc2, speed, address);
            port.send("Moving rear at: " + String(speed));
        
        } else if (action == "s") {
            motionControl::stop(*ptr_rc1, *ptr_rc2, address);
            port.send("Motors Stopped!");

        } else if (action == "v4") {
            float v1 = argument(message, 1);
            float v2 = argument(message, 2);
            float v3 = argument(message, 3);
            float v4 = argument(message, 4);
            motionControl::individualControl(*ptr_rc1, *ptr_rc2, v1, v2, v3, v4, address);
            port.send("v1: " + String(v1));
            port.send("v2: " + String(v2));
            port.send("v3: " + String(v3));
            port.send("v4: " + String(v4));

        } else if (action == "dc") {
            float vl = argument(message, 1);
            float vr = argument(message, 2);
            motionControl::differentialControl(*ptr_rc1, *ptr_rc2, vl, vr, address);
            port.send("vl: " + String(vl));
            port.send("vr: " + String(vr));

           

        } else if (action == "se1") {
            int32_t value = argument(message, 1);
            ptr_rc1->SetEncM1(address, value);
            port.send("Encoder 1 setup to: " + String(value));

        } else if (action == "se2") {
            int32_t value = argument(message, 1);
            ptr_rc1->SetEncM2(address, value);
            port.send("Encoder 2 setup to: " + String(value));

        } else if (action == "se3") {
            int32_t value = argument(message, 1);
            ptr_rc2->SetEncM1(address, value);
            port.send("Encoder 3 setup to: " + String(value));

        } else if (action == "se4") {
            int32_t value = argument(message, 1);
            ptr_rc2->SetEncM2(address, value);
            port.send("Encoder 4 setup to: " + String(value));
            
        } else if (action == "rese") {
            ptr_rc1->ResetEncoders(address);
            ptr_rc2->ResetEncoders(address);
            port.send("Encoders reseted!");
            
          

        } else if (action == "re1") {
            port.send("Encoder 1: " + String(ptr_rc1->ReadEncM1(address)));

        } else if (action == "re2") {
            port.send("Encoder 2: " + String(ptr_rc1->ReadEncM2(address)));

        } else if (action == "re3") {
            port.send("Encoder 3: " + String(ptr_rc2->ReadEncM1(address)));

        } else if (action == "re4") {
            port.send("Encoder 4: " + String(ptr_rc2->ReadEncM2(address)));

        } else if (action == "re") {
            port.send("Encoder 1: " + String(ptr_rc1->ReadEncM1(address)));
            port.send("Encoder 2: " + String(ptr_rc1->ReadEncM2(address)));
            port.send("Encoder 3: " + String(ptr_rc2->ReadEncM1(address)));
            port.send("Encoder 4: " + String(ptr_rc2->ReadEncM2(address)));

        } else if (action == "t") {
            uint16_t temp1;
            uint16_t temp2;
            uint16_t temp3;
            uint16_t temp4;
            ptr_rc1->ReadTemp(address, temp1);
            ptr_rc1->ReadTemp(address,temp1);
            ptr_rc1->ReadTemp2(address,temp2);
            ptr_rc2->ReadTemp(address,temp3);
            ptr_rc2->ReadTemp2(address,temp4);
            port.send("temperature 1 is: " + String(convertTemperature(temp1)) + " F");
            port.send("temperature 2 is: " + String(convertTemperature(temp2)) + " F");
            port.send("temperature 3 is: " + String(convertTemperature(temp3)) + " F");
            port.send("temperature 4 is: " + String(convertTemperature(temp4)) + " F");


        } else if (action == "b") {
            port.send("Battery RC1: " + String(ptr_rc1->ReadMainBatteryVoltage(address)));
            port.send("Battery RC2: " + String(ptr_rc2->ReadMainBatteryVoltage(address)));

        } else if (action == "v") {
            char version[32];
            if (ptr_rc1->ReadVersion(address, version)) {
            port.send(version);
            }

        } else if (action == "ss") {
            port.send("Speed M1: " + String(ptr_rc1->ReadSpeedM1(address)));
            port.send("Speed M2: " + String(ptr_rc1->ReadSpeedM2(address)));
            port.send("Speed M3: " + String(ptr_rc2->ReadSpeedM1(address)));
            port.send("Speed M4: " + String(ptr_rc2->ReadSpeedM2(address)));

        } else {

            port.send("Ops, '" + message + "' command not found!");
            for (int i = 0; i < 10; ++i) {
            port.send(token(message, i));
            }
        }
    } 

    float convertTemperature(uint16_t rawTemperature) {
        float realTemperature = (1.8)*(rawTemperature - 273.15) + 32;
    return realTemperature;
    }
        
}

#endif