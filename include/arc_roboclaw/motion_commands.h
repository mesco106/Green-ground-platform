#ifndef MOTION_COMMANDS_H
#define MOTION_COMMANDS_H

#include <RoboClaw.h>

namespace motionCommands{
    void forward(RoboClaw m1, RoboClaw m2, int speed, char address);
    void backward(RoboClaw m1, RoboClaw m2, int speed, char address);
    void left(RoboClaw m1, RoboClaw m2, int speed, char address);
    void right(RoboClaw m1, RoboClaw m2, int speed, char address);
    void individualControl(RoboClaw m1, RoboClaw m2, int v1, int v2, int v3, int v4, char address);
    void differentialControl(RoboClaw m1, RoboClaw m2, int vl, int vr, char address);
    void stop(RoboClaw m);
} 

#endif