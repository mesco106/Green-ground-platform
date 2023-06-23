#include <RoboClaw.h>

#include "Arduino.h"
#include "SerialPort.h"
#include "Gpio.h"

//#define DEBUG

#ifdef DEBUG
  #define debug(x) port.send(x)
#else
  #define debug(x)
#endif



#define p_channel1 7
#define p_channel2 8
#define p_channel3 9
#define p_channel4 10
#define p_channel5 11
#define p_channel6 12


void forward(RoboClaw m1, RoboClaw m2, int speed, char address);
void backward(RoboClaw m1, RoboClaw m2, int speed, char address);
void right(RoboClaw m1, RoboClaw m2, int speed, char address);
void left(RoboClaw m1, RoboClaw m2, int speed, char address);
void individualControl(RoboClaw m1, RoboClaw m2, int v1, int v2, int v3, int v4, char address);
void differentialControl(RoboClaw m1, RoboClaw m2, int vl, int vr, char address);
void stop(RoboClaw m1, RoboClaw m2, char address);
void run(const String &message);
void execution(RoboClaw front, RoboClaw rear);
float convertTemperature(uint16_t rawTemperature);
void calibration();
void read();
void print();

int r1;
int r2;
const char address = 0x80;

RoboClaw roboclaw1(&Serial3, 10000);
RoboClaw roboclaw2(&Serial4, 10000);
SerialPort port(115200, run);
Gpio led(LED_BUILTIN);

void setup() {
  Serial.begin(115200);
  
  //RECEIVER
  pinMode(p_channel1, INPUT);
  pinMode(p_channel2, INPUT);
  pinMode(p_channel3, INPUT);
  pinMode(p_channel4, INPUT);
  pinMode(p_channel5, INPUT);
  pinMode(p_channel6, INPUT);

  //ROBOCLAW
  roboclaw1.begin(38400);
  roboclaw2.begin(38400);

  //SERIAL PORT
  port.setup();

  //LED
  led.setup();
}

void loop() {

  bool state = true;
  bool *p_state = &state;

  int trigger = digitalRead(p_channel5);
  
  if (trigger == true)
    state = true;
  else 
    state = false;

  if (state == true)
  execution(roboclaw1, roboclaw2);
  else {
    while (1){
      port.update();
      if (*p_state == false)
       break;
    }
  }
}

void read() {
   r1 = pulseIn(p_channel1, HIGH, 20000); 
   r2 = pulseIn(p_channel2, HIGH, 20000); 
  }
void calibration(){

    int * channel1 = &r1;
    int * channel2 = &r2;

    if (*channel1 > 2000) *channel1 = 2000;
    if (*channel1 < 1000) *channel1 = 1000;
    if (*channel2 > 2000) *channel2 = 2000;
    if (*channel2 < 1000) *channel2 = 1000;
  }
void execution(RoboClaw front, RoboClaw rear) {
    int * channel1 = &r1;
    int * channel2 = &r2;
    
    read();
    calibration();

    if (*channel2 > (1600)) {     
      int y_speed = map(*channel2, (1600), 2000, 0, 126);
      forward(front, rear, y_speed, address);
    } else if (*channel2 < (1400)){
      int y_speed = map(*channel2, (1400), 1000, 0, 126);
      backward(front, rear, y_speed, address);
    } else if (*channel2 >= (1400) && *channel2 <= (1600)){
      stop(front, rear, address);
    }

    if (*channel1 > (1650)) {
      int x_speed = map(*channel1, (1650), 2000, 0, 126);
      right(front, rear, x_speed, address);

    } else if (*channel1 < (1350)){
      int x_speed = map(*channel1, (1350), 1000, 0, 126);
      left(front, rear, x_speed, address);

    } else if (*channel2 >= (1350) && *channel2 <= (1650)){
      stop(front, rear, address);
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

void run(const String &message) {
    auto action = command(message);

    if (action = "S") {
      auto linear_vel = argument(message,0);
      auto angular_vel = argument(message, 1);
      angular_vel = map(angular_vel, -1, 1, -1.5, 1.5);
      float wheelsGap = 0.711; // m
      float wheelRadius = 0.33; // m
      float V = map(linear_vel, 0, 1.5, 0, 1.866); // m/s
      float W = map(angular_vel, 0, 1.5, 0, 2.624); // rad/sec
      float wr = (V + 0.5*wheelsGap*W)/wheelRadius;
      float wl = (V - 0.5*wheelsGap*W)/wheelRadius;
      float vr = map(wr, 0, 5.65, 0, 126);
      float vl = map(wl, 0, 5.65, 0, 126);
      if (vr > 126)
        vr = 126;
      if (vr < -126)
        vr = -126;

      if (vl > 126)
        vl = 126;
      if (vl < -126)
        vl = -126;
      differentialControl(roboclaw1, roboclaw2, vl, vr, address);

    }else if (action == "on") {
        led.on();
    } else if (action == "off") {
        led.off();
    } else if (action == "hi") {
        port.send("Hi there!");
    } else if (action == "fo") {
        float speed = argument(message, 1);
        forward(roboclaw1, roboclaw2, speed, address);
        debug("Moving forwards at: " + String(speed));
    } else if (action == "ba") {
        float speed = argument(message, 1);
        backward(roboclaw1, roboclaw2, speed, address);
        debug("Moving backwards at: " + String(speed));
    } else if (action == "le") {
        float speed = argument(message, 1);
        left(roboclaw1, roboclaw2, speed, address);
        debug("Moving left at: " + String(speed));
    } else if (action == "ri") {
        float speed = argument(message, 1);
        right(roboclaw1, roboclaw2, speed, address);
        debug("Moving right at: " + String(speed));
    } else if (action == "s") {
        stop(roboclaw1, roboclaw2, address);
        debug("motors stopped");
    } else if (action == "ri") {
        float speed = argument(message, 1);
        right(roboclaw1, roboclaw2, speed, address);
        debug("Moving right at: " + String(speed));
    } else if (action == "v4") {
        float v1 = argument(message, 1);
        float v2 = argument(message, 2);
        float v3 = argument(message, 3);
        float v4 = argument(message, 4);
        individualControl(roboclaw1, roboclaw2, v1, v2, v3, v4, address);
        debug("v1: " + String(v1));
        debug("v2: " + String(v2));
        debug("v3: " + String(v3));
        debug("v4: " + String(v4));
    } else if (action == "dc") {
        float vl = argument(message, 1);
        float vr = argument(message, 2);
        differentialControl(roboclaw1, roboclaw2, vl, vr, address);
        debug("vl: " + String(vl));
        debug("vr: " + String(vr));
    } else if (action == "se1") {
        int32_t value = argument(message, 1);
        roboclaw1.SetEncM1(address, value);
        debug("Encoder 1 setup to: " + String(value));
    } else if (action == "se2") {
        int32_t value = argument(message, 1);
        roboclaw1.SetEncM2(address, value);
        debug("Encoder 2 setup to: " + String(value));
    } else if (action == "se3") {
        int32_t value = argument(message, 1);
        roboclaw2.SetEncM1(address, value);
        debug("Encoder 3 setup to: " + String(value));
    } else if (action == "se4") {
        int32_t value = argument(message, 1);
        roboclaw2.SetEncM2(address, value);
        debug("Encoder 4 setup to: " + String(value));
    } else if (action == "rese") {
        roboclaw1.ResetEncoders(address);
        roboclaw2.ResetEncoders(address);
        debug("Encoders reseted!");
    } else if (action == "re1") {
        debug("Encoder 1: " + String(roboclaw1.ReadEncM1(address)));
    } else if (action == "re2") {
        debug("Encoder 2: " + String(roboclaw1.ReadEncM2(address)));
    } else if (action == "re3") {
        debug("Encoder 3: " + String(roboclaw2.ReadEncM1(address)));
    } else if (action == "re4") {
        debug("Encoder 4: " + String(roboclaw2.ReadEncM2(address)));

    } else if (action == "re") {
        debug("Encoders : " + String(roboclaw2.ReadEncM2(address)) + ' '
                                + String(roboclaw2.ReadEncM1(address)) + ' '
                                + String(roboclaw2.ReadEncM1(address)) + ' '
                                + String(roboclaw2.ReadEncM2(address)));
    } else if (action == "t") {
        uint16_t temp1;
        uint16_t temp2;
        uint16_t temp3;
        uint16_t temp4;
        roboclaw1.ReadTemp(address, temp1);
        roboclaw1.ReadTemp(address,temp1);
        roboclaw1.ReadTemp2(address,temp2);
        roboclaw2.ReadTemp(address,temp3);
        roboclaw2.ReadTemp2(address,temp4);
        debug("Temperature " + String(convertTemperature(temp1)) + ' '
                                 + String(convertTemperature(temp2)) + ' '
                                 + String(convertTemperature(temp3)) + ' '
                                 + String(convertTemperature(temp4)));
    } else if (action == "b") {
        debug("Battery " + String(roboclaw1.ReadMainBatteryVoltage(address)) + ' '
                             + String(roboclaw2.ReadMainBatteryVoltage(address)));
    } else if (action == "v") {
        char version[32];
        if (roboclaw1.ReadVersion(address, version))
          debug(String("Version ") + version);
    } else if (action == "ss") {
        debug("Speed " + String(roboclaw1.ReadSpeedM1(address)) + ' '
                           + String(roboclaw1.ReadSpeedM2(address)) + ' '
                           + String(roboclaw2.ReadSpeedM1(address)) + ' '
                           + String(roboclaw2.ReadSpeedM2(address)));
    } else {
      debug("Ops, '" + message + "' command not found!");
    }
} 

float convertTemperature(uint16_t rawTemperature) {
  float realTemperature = (1.8)*(rawTemperature - 273.15) + 32;
  return realTemperature;
}