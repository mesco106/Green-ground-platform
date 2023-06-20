#include "Arduino.h"
#include "SerialPort.h"
#include "Gpio.h"
#include <RoboClaw.h>
#include <motion_control.h>
#include <remote_control.h>

#define address 0x80

#define p_channel1 7
#define p_channel2 8
#define p_channel3 9
#define p_channel4 10
#define p_channel5 11
#define p_channel6 12

float convertTemperature(float rawTemperature);
void run(const String &message);

SerialPort port(115200, run);
Gpio led(LED_BUILTIN);
RoboClaw roboclaw1(&Serial3, 10000);
RoboClaw roboclaw2(&Serial4, 10000);



void setup() {

  pinMode(p_channel1, INPUT);
  pinMode(p_channel2, INPUT);
  pinMode(p_channel3, INPUT);
  pinMode(p_channel4, INPUT);
  pinMode(p_channel5, INPUT);
  pinMode(p_channel6, INPUT);

  roboclaw1.begin(38400);
  roboclaw2.begin(38400);
  port.setup();
  led.setup();

}

void loop() {
  int channel5 = pulseIn(p_channel5, HIGH, 36000);

  if (channel5 > 1800){
    remoteControl::execution(roboclaw1, roboclaw2);
  } else if (channel5 >= 1000 && channel5 <= 1800) {
    port.update();
  } else if (channel5 < 1000){
    Serial.println("unused mode");
  }
  
}

void run(const String &message) {
  
  auto action = command(message);
  

   if (action == "on") {
    led.on();


  } else if (action == "off") {
    led.off();


  } else if (action == "hi") {
    port.send("Hi there!");

    /*__________________________MOTOR COMMANDS_________________________________*/

  } else if (action == "fo") {
    float speed = argument(message, 1);
    motionControl::forward(roboclaw1, roboclaw2, speed, address);
    port.send("Moving forwards at: " + String(speed));

  } else if (action == "ba") {
    float speed = argument(message, 1);
    motionControl::backward(roboclaw1, roboclaw2, speed, address);
    port.send("Moving backwards at: " + String(speed));

  } else if (action == "le") {
    float speed = argument(message, 1);
    motionControl::left(roboclaw1, roboclaw2, speed, address);
    port.send("Moving left at: " + String(speed));

  } else if (action == "ri") {
    float speed = argument(message, 1);
    motionControl::right(roboclaw1, roboclaw2, speed, address);
    port.send("Moving right at: " + String(speed));
  
  } else if (action == "s") {
    motionControl::stop(roboclaw1, roboclaw2, address);
    port.send("Motors Stopped!");

  } else if (action == "v4") {
    float v1 = argument(message, 1);
    float v2 = argument(message, 2);
    float v3 = argument(message, 3);
    float v4 = argument(message, 4);
    motionControl::individualControl(roboclaw1, roboclaw2, v1, v2, v3, v4, address);
    port.send("v1: " + String(v1));
    port.send("v2: " + String(v2));
    port.send("v3: " + String(v3));
    port.send("v4: " + String(v4));

  } else if (action == "dc") {
    float vl = argument(message, 1);
    float vr = argument(message, 2);
    motionControl::differentialControl(roboclaw1, roboclaw2, vl, vr, address);
    port.send("vl: " + String(vl));
    port.send("vr: " + String(vr));

    /*__________________________SET COMMANDS_______________________________*/

  } else if (action == "se1") {
    int32_t value = argument(message, 1);
    roboclaw1.SetEncM1(address, value);
    port.send("Encoder 1 setup to: " + String(value));

  } else if (action == "se2") {
    int32_t value = argument(message, 1);
    roboclaw1.SetEncM2(address, value);
    port.send("Encoder 2 setup to: " + String(value));

  } else if (action == "se3") {
    int32_t value = argument(message, 1);
    roboclaw2.SetEncM1(address, value);
    port.send("Encoder 3 setup to: " + String(value));

  } else if (action == "se4") {
    int32_t value = argument(message, 1);
    roboclaw2.SetEncM2(address, value);
    port.send("Encoder 4 setup to: " + String(value));
    
  } else if (action == "rese") {
    roboclaw1.ResetEncoders(address);
    roboclaw2.ResetEncoders(address);
    port.send("Encoders reseted!");
    
    /*__________________________READ COMMAND_________________________________*/

  } else if (action == "re1") {
    port.send("Encoder 1: " + String(roboclaw1.ReadEncM1(address)));

  } else if (action == "re2") {
    port.send("Encoder 2: " + String(roboclaw1.ReadEncM2(address)));

  } else if (action == "re3") {
    port.send("Encoder 3: " + String(roboclaw2.ReadEncM1(address)));

  } else if (action == "re4") {
    port.send("Encoder 4: " + String(roboclaw2.ReadEncM2(address)));

  } else if (action == "re") {
    port.send("Encoder 1: " + String(roboclaw1.ReadEncM1(address)));
    port.send("Encoder 2: " + String(roboclaw1.ReadEncM2(address)));
    port.send("Encoder 3: " + String(roboclaw2.ReadEncM1(address)));
    port.send("Encoder 4: " + String(roboclaw2.ReadEncM2(address)));

  } else if (action == "t") {
    uint16_t temp1;
    uint16_t temp2;
    uint16_t temp3;
    uint16_t temp4;
    roboclaw1.ReadTemp(address, temp1);
    roboclaw1.ReadTemp2(address, temp2);
    roboclaw2.ReadTemp(address, temp3);
    roboclaw2.ReadTemp2(address, temp4);
    port.send("temperature 1 is: " + String(convertTemperature(temp1)) + " F");
    port.send("temperature 2 is: " + String(convertTemperature(temp2)) + " F");
    port.send("temperature 3 is: " + String(convertTemperature(temp3)) + " F");
    port.send("temperature 4 is: " + String(convertTemperature(temp4)) + " F");


  } else if (action == "b") {
    port.send("Battery RC1: " + String(roboclaw1.ReadMainBatteryVoltage(address)));
    port.send("Battery RC2: " + String(roboclaw2.ReadMainBatteryVoltage(address)));

  } else if (action == "v") {
    char version[32];
    if (roboclaw1.ReadVersion(address, version)) {
      port.send(version);
    }

  } else if (action == "ss") {
    port.send("Speed M1: " + String(roboclaw1.ReadSpeedM1(address)));
    port.send("Speed M2: " + String(roboclaw1.ReadSpeedM2(address)));
    port.send("Speed M3: " + String(roboclaw2.ReadSpeedM1(address)));
    port.send("Speed M4: " + String(roboclaw2.ReadSpeedM2(address)));

  } else {
    /*__________________________ERROR COMMANDS_______________________________*/
    port.send("Ops, '" + message + "' command not found!");
    for (int i = 0; i < 10; ++i) {
      port.send(token(message, i));
    }
  }
}

float convertTemperature(float rawTemperature) {
  float realTemperature = (1.8)*(rawTemperature - 273.15) + 32;
  return realTemperature;
}
