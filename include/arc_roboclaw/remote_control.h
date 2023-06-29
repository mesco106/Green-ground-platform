#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

const int address = 0x80;

#include "Arduino.h"
#include <RoboClaw.h>

namespace remoteControl {
    void setup();
    void execution(RoboClaw front, RoboClaw rear);

}


    /* IN DEVELOPMENT
        chan_1_read = pulseIn(channel1,HIGH,36000);
        chan_2_read = pulseIn(channel2,HIGH,36000);

        if (chan_1_read > 2000) chan_1_read = 2000;
        if (chan_1_read < 1000) chan_1_read = 1000;

        if (chan_2_read > 2000) chan_2_read = 2000;
        if (chan_2_read < 1000) chan_2_read = 1000;

        float wheelsGap = 0.711; // m
        float wheelRadius = 0.33; // m
        float V = map(chan_1_read, 1000.0, 2000.0, -1.866, 1.866); // m/s
        float W = map(chan_2_read, 1000.0, 2000.0, -2.624, 2.624); // rad/sec


        float wr = (V + 0.5*wheelsGap*W)/wheelRadius;
        float wl = (V - 0.5*wheelsGap*W)/wheelRadius;
        float vr = map(wr, 0, 5.65, 0, 126);
        float vl = map(wl, 0, 5.65, 0, 126);

        Serial.println(vr);
        Serial.println(vl);

      if (vr > 126)
        vr = 126;
      if (vr < -126)
        vr = -126;

      if (vl > 126)
        vl = 126;
      if (vl < -126)
        vl = -126;

    */




























    // int x_speed;
    // int y_speed;

    // void execution(RoboClaw front, RoboClaw rear) {
    // int channel1;
    // int channel2;

    // channel2 = pulseIn(p_channel2, HIGH, 20000); 
    // if (channel2 > 2000) channel2 = 2000;
    // if (channel2 < 1000) channel2 = 1000;

    // if (channel2 > 1500) {     
    //   y_speed = map(channel2, 1550, 2000, 0, 126);
    //   if (y_speed < 0)
    //     y_speed = 0;
    //   motionCommands::forward(front, rear, y_speed, address);

    // } else if (channel2 < 1450){
    //   y_speed = map(channel2, 1400, 1000, 0, 126);
    //   if (y_speed < 0)
    //     y_speed = 0;      
    //   motionCommands::backward(front, rear, y_speed, address);
    // // } else if (channel2 >= 1400 && channel2 <= 1600){
    // //   // forward(front, rear, 2, address);
    // }


    // channel1 = pulseIn(p_channel1, HIGH, 20000); 
    // if (channel1 > 2000) channel1 = 2000;
    // if (channel1 < 1000) channel1 = 1000;

    // if (channel1 > 1550) {
    //   x_speed = map(channel1, 1600, 2000, 0, 126);
    //   if (x_speed < 0)
    //     x_speed = 0;
    //   motionCommands::rear(front, rear, x_speed, address);
      

    // } else if (channel1 < 1450){
    //   x_speed = map(channel1, 1400, 1000, 0, 126);
    //   if (x_speed < 0)
    //     x_speed = 0;
    //   motionCommands::front(front, rear, x_speed, address);

    // //  } else if (channel1 >= 1400 && channel1 <= 1600){
    // //     // rear(front, rear, 2, address);
    
   //  }

 // }
//}

#endif