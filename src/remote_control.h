#include <Arduino.h>
#include <RoboClaw.h>
#include <motion_control.h>

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#define p_channel1 7
#define p_channel2 8

#define address 0x80

namespace remoteControl {
  int r1;
  int r2;
  
  void read() {
   r1 = pulseIn(p_channel1, HIGH, 36000); 
   r2 = pulseIn(p_channel2, HIGH, 36000); 
  }

  void calibration(){

    int * channel1 = &r1;
    int * channel2 = &r2;

    if (*channel1 > 2000) *channel1 = 2000;
    if (*channel1 < 1000) *channel1 = 1000;
    if (*channel2 > 2000) *channel2 = 2000;
    if (*channel2 < 1000) *channel2 = 1000;
  }

  void execution(RoboClaw front, RoboClaw rear, int x_sensibility = 50, int y_sensibility = 50, int treshold = 50) {
    int * channel1 = &r1;
    int * channel2 = &r2;
    
    read();
    calibration();

    if (*channel2 > (1500+y_sensibility+treshold)) {

      int y_speed = map(*channel2, (1500+y_sensibility), 2000, 0, 120);
      motionControl::forward(front, rear, y_speed, address);

    } else if (*channel2 < (1500-y_sensibility-treshold)){

      int y_speed = map(*channel2, (1500-y_sensibility), 1000, 0, 126);
      motionControl::backward(front, rear, y_speed, address);

    } else if (*channel2 > (1500-y_sensibility-treshold) && *channel2 < (1500+y_sensibility+treshold)){

      motionControl::stop(front, rear, address);
    }

    if (*channel1 > (1500+y_sensibility+treshold)) {

      int x_speed = map(*channel1, (1500+x_sensibility), 2000, 0, 126);
      motionControl::right(front, rear, x_speed, address);

    } else if (*channel1 < (1500-y_sensibility-treshold)){

      int x_speed = map(*channel1, (1500-x_sensibility), 1000, 0, 126);
      motionControl::left(front, rear, x_speed, address);

    } else if (*channel2 > (1500-y_sensibility-treshold) && *channel2 < (1500+y_sensibility+treshold)){

      motionControl::stop(front, rear, address);
    }
  }

  void print(){
    int * channel1 = &r1;
    int * channel2 = &r2;

    read();
    calibration();
    Serial.print(*channel1);
    Serial.print("  |  ");
    Serial.println(*channel2);
  }
}

#endif