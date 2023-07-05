#include <RoboClaw.h>

#include "Arduino.h"
#include "remote_control.h"
#include "SerialPort.h"
#include "Gpio.h"

//#define DEBUG

// #ifdef DEBUG
//   #define debug(x) port.send(x)
// #else
//   #define debug(x)
// #endif

int i = 0;

#define InputPin 5
#define LinearActuator 12
#define p_channel1 8
#define p_channel2 7
#define p_channel3 9

#define enA 19 
#define in1 20
#define in2 21

#define Kp_m1_f 0.23384
#define Ki_m1_f 0.03053
#define Kd_m1_f 0.0
#define qpps_m1_f 105000

#define Kp_m2_f 0.27255
#define Ki_m2_f 0.03382
#define Kd_m2_f 0.0
#define qpps_m2_f 92625

#define Kp_m1_r 0.25646
#define Ki_m1_r 0.03182
#define Kd_m1_r 0.0
#define qpps_m1_r 99937

#define Kp_m2_r 0.29799
#define Ki_m2_r 0.03642
#define Kd_m2_r 0.0
#define qpps_m2_r 85687

// void forward(RoboClaw m1, RoboClaw m2, int speed, char address);
// void backward(RoboClaw m1, RoboClaw m2, int speed, char address);
// void rear(RoboClaw m1, RoboClaw m2, int speed, char address);
// void front(RoboClaw m1, RoboClaw m2, int speed, char address);
// void individualControl(RoboClaw m1, RoboClaw m2, int v1, int v2, int v3, int v4, char address);
// void differentialControl(RoboClaw m1, RoboClaw m2, int vl, int vr, char address);
// void stop(RoboClaw m1);
// void run(const String &message);
// void ActuateSolarPanels();
// void calibration();
// void read();
// void execution(RoboClaw front, RoboClaw rear);
// float convertTemperature(uint16_t rawTemperature);
// void print();


RoboClaw roboclaw1(&Serial4, 10000);
RoboClaw roboclaw2(&Serial3, 10000);
// SerialPort port(115200, run);
Gpio led(LED_BUILTIN);

void setup() {
  Serial.begin(9600);
  
  //RECEIVER
  pinMode(p_channel1, INPUT);
  pinMode(p_channel2, INPUT);
  pinMode(p_channel3, INPUT);
  
  //ROBOCLAW
  roboclaw1.begin(38400);
  roboclaw2.begin(38400);

  roboclaw1.SetM1VelocityPID(address,Kd_m1_f,Kp_m1_f,Ki_m1_f, qpps_m1_f);
  roboclaw1.SetM2VelocityPID(address,Kd_m2_f,Kp_m2_f,Ki_m2_f, qpps_m2_f);
  roboclaw2.SetM1VelocityPID(address,Kd_m1_r,Kp_m1_r,Ki_m1_r, qpps_m1_r);
  roboclaw2.SetM2VelocityPID(address,Kd_m2_r,Kp_m2_r,Ki_m2_r, qpps_m2_r);

  //SERIAL PORT
  // port.setup();

  //LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  led.setup();

  //LINEAR ACTUATOR
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  

}

void loop() {
  
  remoteControl::execution(roboclaw1, roboclaw2);

  // ActuateSolarPanels();


  // i++;
  // //Serial.println(i);

  // int state;
  // int *p_state = &state;
  

  // int read = digitalRead(InputPin);

  // if (read == 1) {
  //   //digitalWrite(LED_BUILTIN, HIGH);
  //   state = 1;
  // }else{
  //   //digitalWrite(LED_BUILTIN, LOW);
  //   state = 0;
  // }
 

  // if (state == 1)
  // execution(roboclaw1, roboclaw2);
  // //Serial.println(read);
  // else {
  //   while (1){
  //     ;
  //     //Serial.println(read);
  //     if (*p_state == false)
  //      break;
  //   }
  // }
}


//void execution(RoboClaw front, RoboClaw rear) {
  //   int channel1;
  //   int channel2;



  //     channel2 = pulseIn(p_channel2, HIGH, 20000); 
  //   if (channel2 > 2000) channel2 = 2000;
  //   if (channel2 < 1000) channel2 = 1000;



  // //_______________________________________________

  //   if (channel2 > 1500) {     
  //     y_speed = map(channel2, 1550, 2000, 0, 126);
  //     if (y_speed < 0)
  //       y_speed = 0;
  //     forward(front, rear, y_speed, address);

  //   } else if (channel2 < 1450){
  //     y_speed = map(channel2, 1400, 1000, 0, 126);
  //     if (y_speed < 0)
  //       y_speed = 0;      
  //     backward(front, rear, y_speed, address);
  //   // } else if (channel2 >= 1400 && channel2 <= 1600){
  //   //   // forward(front, rear, 2, address);
  //   }


  //   channel1 = pulseIn(p_channel1, HIGH, 20000); 
  //   if (channel1 > 2000) channel1 = 2000;
  //   if (channel1 < 1000) channel1 = 1000;

  //   if (channel1 > 1550) {
  //     x_speed = map(channel1, 1600, 2000, 0, 126);
  //     if (x_speed < 0)
  //       x_speed = 0;
  //     rear(front, rear, x_speed, address);
      

  //   } else if (channel1 < 1450){
  //     x_speed = map(channel1, 1400, 1000, 0, 126);
  //     if (x_speed < 0)
  //       x_speed = 0;
  //     front(front, rear, x_speed, address);

  //   //  } else if (channel1 >= 1400 && channel1 <= 1600){
  //   //     // rear(front, rear, 2, address);
    
  //    }

  //   Serial.print(channel1);
  //   Serial.print("  |  ");
  //   Serial.println(channel2);
  // }

//void print(){
  //   int * channel1 = &r1;
  //   int * channel2 = &r2;

  //   read();
  //   calibration();
  //   Serial.print(*channel1);
  //   Serial.print("  |  ");
  //   Serial.println(*channel2);
  // }
// void forward(RoboClaw m1, RoboClaw m2, int speed, char address) {
//     m1.ForwardM1(address, speed);
//     m1.ForwardM2(address, speed);
//     m2.ForwardM1(address, speed);
//     m2.ForwardM2(address, speed);
// }
// void backward(RoboClaw m1, RoboClaw m2, int speed, char address) {
//     m1.BackwardM1(address, speed);
//     m1.BackwardM2(address, speed);
//     m2.BackwardM1(address, speed);
//     m2.BackwardM2(address, speed);
// }
// void rear(RoboClaw m1, RoboClaw m2, int speed, char address) {
//     m1.ForwardM1(address, speed);
//     m1.BackwardM2(address, speed);
//     m2.ForwardM1(address, speed);
//     m2.BackwardM2(address, speed);
// }
// void front(RoboClaw m1, RoboClaw m2, int speed, char address) {
//     m1.BackwardM1(address, speed);
//     m1.ForwardM2(address, speed);
//     m2.BackwardM1(address, speed);
//     m2.ForwardM2(address, speed);
// }
// void individualControl(RoboClaw m1, RoboClaw m2, int v1, int v2, int v3, int v4, char address){
//     if (v1 > 0) {m1.ForwardM1(address, v1);}
//     else if (v1 < 0) {m1.BackwardM1(address, v1*(-1));}
//     else if (v1 == 0) { m1.ForwardM1(address, 0);}

//     if (v2 > 0) {m1.ForwardM2(address, v2);}
//     else if (v2 < 0) {m1.BackwardM2(address, v2*(-1));}
//     else if (v2 == 0) { m1.ForwardM2(address, 0);}

//     if (v3 > 0) {m2.ForwardM1(address, v3);}
//     else if (v3 < 0) {m2.BackwardM1(address, v3*(-1));}
//     else if (v3 == 0) { m2.ForwardM1(address, 0);}
    
//     if (v4 > 0) {m2.ForwardM2(address, v4);}
//     else if (v4 < 0) {m2.BackwardM2(address, v4*(-1));}
//     else if (v4 == 0) { m2.ForwardM2(address, 0);}

// }
// void differentialControl(RoboClaw m1, RoboClaw m2, int vl, int vr, char address){
//     if (vl > 0) {
//         m1.ForwardM1(address, vl);
//         m2.ForwardM1(address, vl);
//     } else if (vl < 0) {
//         m1.BackwardM1(address, vl*(-1));
//         m2.BackwardM1(address, vl*(-1));
    
//     } else if (vl == 0) { 
//         m1.BackwardM1(address, 0);
//         m2.BackwardM1(address, 0);
//     }

//     if (vr > 0) {
//         m1.ForwardM2(address, vr);
//         m2.ForwardM2(address, vr);
//     } else if (vr < 0) {
//         m1.BackwardM2(address, vr*(-1));
//         m2.BackwardM2(address, vr*(-1));
    
//     } else if (vr == 0) { 
//         m1.ForwardM2(address, 0);
//         m2.ForwardM2(address, 0);
//     }

// }
// void stop(RoboClaw m) {
//     m.ForwardM1(0x80, 0);
//     m.ForwardM2(0x80, 0);
// }

// void run(const String &message) {
//     auto action = command(message);
//     if (action = "S") {
//       auto linear_vel = argument(message,1);
//       auto angular_vel = argument(message, 2);
//       angular_vel = map(angular_vel, -1, 1, -1.5, 1.5);
//       // float wheelsGap = 0.711; // m
//       // float wheelRadius = 0.33; // m
//       // float V = map(linear_vel, 0, 1.5, 0, 1.866); // m/s
//       // float W = map(angular_vel, 0, 1.5, 0, 2.624); // rad/sec

//       float V = map(linear_vel, 0, 1.5, 0, 126); // m/s
//       float W = map(angular_vel, 0, 1.5, 0, 126); // rad/sec

//       // float wr = (V + 0.5*wheelsGap*W)/wheelRadius;
//       // float wl = (V - 0.5*wheelsGap*W)/wheelRadius;
//       // float vr = map(wr, 0, 5.65, 0, 126);
//       // float vl = map(wl, 0, 5.65, 0, 126);
//       roboclaw1.ForwardM1(address, V);
//       roboclaw1.ForwardM2(address, V);
//       //port.send(V);
//       // port.send(W);

//       // forward(roboclaw1, roboclaw2, vr, address);
//       // if (vr > 126)
//       //   vr = 126;
//       // if (vr < -126)
//       //   vr = -126;

//       // if (vl > 126)
//       //   vl = 126;
//       // if (vl < -126)
//       //   vl = -126;
      
//       // differentialControl(roboclaw1, roboclaw2, vl, vr, address);

//     }
// }
// void run(const String &message) {
//     auto action = command(message);
//     if (action = "S") {
//       auto linear_vel = argument(message,1);
//       auto angular_vel = argument(message, 2);
//       angular_vel = map(angular_vel, -1, 1, -1.5, 1.5);
//       float wheelsGap = 0.711; // m
//       float wheelRadius = 0.33; // m
//       float V = map(linear_vel, 0, 1.5, 0, 1.866); // m/s
//       float W = map(angular_vel, 0, 1.5, 0, 2.624); // rad/sec
//       float wr = (V + 0.5*wheelsGap*W)/wheelRadius;
//       float wl = (V - 0.5*wheelsGap*W)/wheelRadius;
//       float vr = map(wr, 0, 5.65, 0, 126);
//       float vl = map(wl, 0, 5.65, 0, 126);
//       if (vr > 126)
//         vr = 126;
//       if (vr < -126)
//         vr = -126;

//       if (vl > 126)
//         vl = 126;
//       if (vl < -126)
//         vl = -126;
      
//       differentialControl(roboclaw1, roboclaw2, vl, vr, address);

//     }else if (action == "on") {
//         led.on();
//     } else if (action == "off") {
//         led.off();
//     } else if (action == "hi") {
//         port.send("Hi there!");
//     } else if (action == "fo") {
//         float speed = argument(message, 1);
//         forward(roboclaw1, roboclaw2, speed, address);
//         debug("Moving forwards at: " + String(speed));
//     } else if (action == "ba") {
//         float speed = argument(message, 1);
//         backward(roboclaw1, roboclaw2, speed, address);
//         debug("Moving backwards at: " + String(speed));
//     } else if (action == "le") {
//         float speed = argument(message, 1);
//         front(roboclaw1, roboclaw2, speed, address);
//         debug("Moving front at: " + String(speed));
//     } else if (action == "ri") {
//         float speed = argument(message, 1);
//         rear(roboclaw1, roboclaw2, speed, address);
//         debug("Moving rear at: " + String(speed));
//     } else if (action == "s") {
//         stop(roboclaw1);
//         stop(roboclaw2);
//         debug("motors stopped");
//     } else if (action == "ri") {
//         float speed = argument(message, 1);
//         rear(roboclaw1, roboclaw2, speed, address);
//         debug("Moving rear at: " + String(speed));
//     } else if (action == "v4") {
//         float v1 = argument(message, 1);
//         float v2 = argument(message, 2);
//         float v3 = argument(message, 3);
//         float v4 = argument(message, 4);
//         individualControl(roboclaw1, roboclaw2, v1, v2, v3, v4, address);
//         debug("v1: " + String(v1));
//         debug("v2: " + String(v2));
//         debug("v3: " + String(v3));
//         debug("v4: " + String(v4));
//     } else if (action == "dc") {
//         float vl = argument(message, 1);
//         float vr = argument(message, 2);
//         differentialControl(roboclaw1, roboclaw2, vl, vr, address);
//         debug("vl: " + String(vl));
//         debug("vr: " + String(vr));
//     } else if (action == "se1") {
//         int32_t value = argument(message, 1);
//         roboclaw1.SetEncM1(address, value);
//         debug("Encoder 1 setup to: " + String(value));
//     } else if (action == "se2") {
//         int32_t value = argument(message, 1);
//         roboclaw1.SetEncM2(address, value);
//         debug("Encoder 2 setup to: " + String(value));
//     } else if (action == "se3") {
//         int32_t value = argument(message, 1);
//         roboclaw2.SetEncM1(address, value);
//         debug("Encoder 3 setup to: " + String(value));
//     } else if (action == "se4") {
//         int32_t value = argument(message, 1);
//         roboclaw2.SetEncM2(address, value);
//         debug("Encoder 4 setup to: " + String(value));
//     } else if (action == "rese") {
//         roboclaw1.ResetEncoders(address);
//         roboclaw2.ResetEncoders(address);
//         debug("Encoders reseted!");
//     } else if (action == "re1") {
//         debug("Encoder 1: " + String(roboclaw1.ReadEncM1(address)));
//     } else if (action == "re2") {
//         debug("Encoder 2: " + String(roboclaw1.ReadEncM2(address)));
//     } else if (action == "re3") {
//         debug("Encoder 3: " + String(roboclaw2.ReadEncM1(address)));
//     } else if (action == "re4") {
//         debug("Encoder 4: " + String(roboclaw2.ReadEncM2(address)));

//     } else if (action == "re") {
//         debug("Encoders : " + String(roboclaw2.ReadEncM2(address)) + ' '
//                                 + String(roboclaw2.ReadEncM1(address)) + ' '
//                                 + String(roboclaw2.ReadEncM1(address)) + ' '
//                                 + String(roboclaw2.ReadEncM2(address)));
//     } else if (action == "t") {
//         uint16_t temp1;
//         uint16_t temp2;
//         uint16_t temp3;
//         uint16_t temp4;
//         roboclaw1.ReadTemp(address, temp1);
//         roboclaw1.ReadTemp(address,temp1);
//         roboclaw1.ReadTemp2(address,temp2);
//         roboclaw2.ReadTemp(address,temp3);
//         roboclaw2.ReadTemp2(address,temp4);
//         debug("Temperature " + String(convertTemperature(temp1)) + ' '
//                                  + String(convertTemperature(temp2)) + ' '
//                                  + String(convertTemperature(temp3)) + ' '
//                                  + String(convertTemperature(temp4)));
//     } else if (action == "b") {
//         debug("Battery " + String(roboclaw1.ReadMainBatteryVoltage(address)) + ' '
//                              + String(roboclaw2.ReadMainBatteryVoltage(address)));
//     } else if (action == "v") {
//         char version[32];
//         if (roboclaw1.ReadVersion(address, version))
//           debug(String("Version ") + version);
//     } else if (action == "ss") {
//         debug("Speed " + String(roboclaw1.ReadSpeedM1(address)) + ' '
//                            + String(roboclaw1.ReadSpeedM2(address)) + ' '
//                            + String(roboclaw2.ReadSpeedM1(address)) + ' '
//                            + String(roboclaw2.ReadSpeedM2(address)));
//     } else {
//       debug("Ops, '" + message + "' command not found!");
//     }
// } 

// void bounder(float &v1, float &v2, float sf = 0.5){
//     v1 = sf*v1;
//     v2 = sf*v2;
// }

// float convertTemperature(uint16_t rawTemperature) {
//   float realTemperature = (1.8)*(rawTemperature - 273.15) + 32;
//   return realTemperature;
// }

// void ActuateSolarPanels(){
//   int input = pulseIn(LinearActuator, HIGH, 36000);
//   if (input > 1500){
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//   } else {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//   }
//   analogWrite(enA, 255);
// }