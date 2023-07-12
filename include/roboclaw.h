#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include <RoboClaw.h>

namespace roboclaw{
    RoboClaw roboclaw1(&Serial3, 10000);
    RoboClaw roboclaw2(&Serial4, 10000);
    RoboClaw roboclaw3(&Serial5, 10000);

    void setup(){
        roboclaw1.begin(115200);
        roboclaw2.begin(115200);
        roboclaw3.begin(115200);
    }
    
}

#endif