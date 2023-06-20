#include <RoboClaw.h>

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

namespace motionControl{
    
    void forward(RoboClaw m1, RoboClaw m2, int speed, char address) {
        m1.ForwardM1(address, speed);
        m1.ForwardM2(address, speed);
        m2.ForwardM1(address, speed);
        m2.ForwardM2(address, speed);
    }
    void backward(RoboClaw m1, RoboClaw m2, int speed, char address) {
        m1.BackwardM1(address, speed);
        m1.BackwardM2(address, speed);
        m2.BackwardM1(address, speed);
        m2.BackwardM2(address, speed);
    }
    void right(RoboClaw m1, RoboClaw m2, int speed, char address) {
        m1.ForwardM1(address, speed);
        m1.BackwardM2(address, speed);
        m2.ForwardM1(address, speed);
        m2.BackwardM2(address, speed);
    }
    void left(RoboClaw m1, RoboClaw m2, int speed, char address) {
        m1.BackwardM1(address, speed);
        m1.ForwardM2(address, speed);
        m2.BackwardM1(address, speed);
        m2.ForwardM2(address, speed);
    }

    void individualControl(RoboClaw m1, RoboClaw m2, int v1, int v2, int v3, int v4, char address){
        m1.ForwardBackwardM1(address, -v1);
        m1.ForwardBackwardM2(address, -v2);
        m2.ForwardBackwardM1(address, -v3);
        m2.ForwardBackwardM2(address, -v4);
    }

    void differentialControl(RoboClaw m1, RoboClaw m2, int vl, int vr, char address){
        m1.ForwardBackwardM1(address, -vl);
        m2.ForwardBackwardM1(address, -vl);
        m1.ForwardBackwardM2(address, -vr);
        m2.ForwardBackwardM2(address, -vr);
    }

    void stop(RoboClaw m1, RoboClaw m2, char address) {
        m1.BackwardM1(address, 0);
        m1.ForwardM2(address, 0);
        m2.BackwardM1(address, 0);
        m2.ForwardM2(address, 0);
    }
}

#endif