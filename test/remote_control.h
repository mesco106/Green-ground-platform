#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <Arduino.h>
#include <RoboClaw.h>
#include <motion_control.h>
#include <init.h>

#define address 0x80

RoboClaw roboclaw1(&Serial3, 10000);
RoboClaw roboclaw2(&Serial4, 10000);


namespace remoteControl {
  int r1;
  int r2;
  
  void read() {
   r1 = pulseIn(reciever::p_channel1, HIGH, 20000); 
   r2 = pulseIn(reciever::p_channel2, HIGH, 20000); 
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
    Serial.print(*channel2);
    Serial.print("   |   ");
    Serial.println(*channel1);

    if (*channel2 > (1600)) {     
      int y_speed = map(*channel2, (1600), 2000, 0, 126);
      motionControl::forward(front, rear, y_speed, address);
    } else if (*channel2 < (1400)){
      int y_speed = map(*channel2, (1400), 1000, 0, 126);
      motionControl::backward(front, rear, y_speed, address);
    } else if (*channel2 >= (1400) && *channel2 <= (1600)){
      motionControl::stop(front, rear, address);
    }

    if (*channel1 > (1650)) {
      int x_speed = map(*channel1, (1650), 2000, 0, 126);
      motionControl::right(front, rear, x_speed, address);

    } else if (*channel1 < (1350)){
      int x_speed = map(*channel1, (1350), 1000, 0, 126);
      motionControl::left(front, rear, x_speed, address);

    } else if (*channel2 >= (1350) && *channel2 <= (1650)){
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