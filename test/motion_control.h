#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <RoboClaw.h>

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
    void rear(RoboClaw m1, RoboClaw m2, int speed, char address) {
        m1.ForwardM1(address, speed);
        m1.BackwardM2(address, speed);
        m2.ForwardM1(address, speed);
        m2.BackwardM2(address, speed);
    }
    void front(RoboClaw m1, RoboClaw m2, int speed, char address) {
        m1.BackwardM1(address, speed);
        m1.ForwardM2(address, speed);
        m2.BackwardM1(address, speed);
        m2.ForwardM2(address, speed);
    }

    void individualControl(RoboClaw m1, RoboClaw m2, int v1, int v2, int v3, int v4, char address){
        if (v1 > 0) {m1.ForwardM1(address, v1);}
        else if (v1 < 0) {m1.BackwardM1(address, v1*(-1));}
        else if (v1 == 0) { m1.ForwardM1(address, 0);}

        if (v2 > 0) {m1.ForwardM2(address, v2);}
        else if (v2 < 0) {m1.BackwardM2(address, v2*(-1));}
        else if (v2 == 0) { m1.ForwardM2(address, 0);}

        if (v3 > 0) {m2.ForwardM1(address, v3);}
        else if (v3 < 0) {m2.BackwardM1(address, v3*(-1));}
        else if (v3 == 0) { m2.ForwardM1(address, 0);}
        
        if (v4 > 0) {m2.ForwardM2(address, v4);}
        else if (v4 < 0) {m2.BackwardM2(address, v4*(-1));}
        else if (v4 == 0) { m2.ForwardM2(address, 0);}

    }
    void differentialControl(RoboClaw m1, RoboClaw m2, int vl, int vr, char address){
        if (vl > 0) {
            m1.ForwardM1(address, vl);
            m2.ForwardM1(address, vl);
        } else if (vl < 0) {
            m1.BackwardM1(address, vl*(-1));
            m2.BackwardM1(address, vl*(-1));
        
        } else if (vl == 0) { 
            m1.BackwardM1(address, 0);
            m2.BackwardM1(address, 0);
        }

        if (vr > 0) {
            m1.ForwardM2(address, vr);
            m2.ForwardM2(address, vr);
        } else if (vr < 0) {
            m1.BackwardM2(address, vr*(-1));
            m2.BackwardM2(address, vr*(-1));
        
        } else if (vr == 0) { 
            m1.ForwardM2(address, 0);
            m2.ForwardM2(address, 0);
        }
    
    }

    void stop(RoboClaw m1, RoboClaw m2, char address) {
        m1.BackwardM1(address, 0);
        m1.ForwardM2(address, 0);
        m2.BackwardM1(address, 0);
        m2.ForwardM2(address, 0);
    }
}

#endif