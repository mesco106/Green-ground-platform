#ifndef INIT_H
#define INIT_H

#include <iostream>
#include <vector>
#include <RoboClaw.h>
#include <roboclaw_cmd.h>

#include "Arduino.h"
#include "SerialPort.h"


namespace reciever
{
    const char p_channel1 = 7;
    const char p_channel2 = 8;
    const char p_channel3 = 9;
    const char p_channel4 = 10;
    const char p_channel5 = 11;
    const char p_channel6 = 12;

    void init(){
        pinMode(p_channel1, INPUT);
        pinMode(p_channel2, INPUT);
        pinMode(p_channel3, INPUT);
        pinMode(p_channel4, INPUT);
        pinMode(p_channel5, INPUT);
        pinMode(p_channel6, INPUT);
    }
}

namespace roboclaw
{   
    RoboClaw roboclaw1 = RoboClaw(&Serial3, 10000);
    RoboClaw roboclaw2 = RoboClaw(&Serial4, 10000);
    
    void init(){
        roboclaw1.begin(38400);
        roboclaw2.begin(38400);
    }

}

#endif