#include "Arduino.h"
#include "SerialPort.h"
#include "Gpio.h"
#include <RoboClaw.h>
#include <motion_control.h>
#include <state.h>
#include <remote_control.h>


void run(const String &message);
void move(float vx, float vy);
void look(float angle);
void stop();
float convertTemperature(float rawTemperature);

using namespace N;

SerialPort port(115200, run);
Gpio led(LED_BUILTIN);
RoboClaw roboclaw1(&Serial3, 10000);
RoboClaw roboclaw2(&Serial4, 10000);

#define address 0x80

namespace reciverPins{
  #define p_channel1 7
  #define p_channel2 8
  #define p_channel3 9
  #define p_channel4 10
  #define p_channel5 11
  #define p_channel6 12
}
  

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

int chan1;
int chan2;

void loop() {
  switch (state(p_channel5)){
  case 0:
    remoteControl::execution(roboclaw1, roboclaw2);
    break;
  case 1:
    port.update();
    break;
  case 2:
    Serial.println(state(p_channel5));
    break;
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


/*__________________________FUNCTIONS_________________________________*/


void move(float vx, float vy) {
  Serial.print("Roboclaw set velocity: ");
  Serial.print(vx);
  Serial.print(" ");
  Serial.println(vy);
}

void look(float angle) {
}

void forward(String Motor, int speed) {
  if (Motor == "m1") {
    roboclaw1.ForwardM1(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "m2") {
    roboclaw1.ForwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "ms") {
    roboclaw1.ForwardM1(address, speed);
    roboclaw1.ForwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  }
}

void backward(String Motor, int speed) {
  if (Motor == "m1") {
    roboclaw1.BackwardM1(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "m2") {
    roboclaw1.BackwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "ms") {
    roboclaw1.BackwardM1(address, speed);
    roboclaw1.BackwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  }
}


void right(float speed) {
  port.send("turning left");
  roboclaw1.ForwardM1(address, speed);
  roboclaw1.BackwardM2(address, speed);
}

void left(float speed) {
  port.send("turning right");
  roboclaw1.ForwardM2(address, speed);
  roboclaw1.BackwardM1(address, speed);
}

void stop() {
  roboclaw1.ForwardM1(address, 0);
  roboclaw1.ForwardM2(address, 0);
  roboclaw2.ForwardM1(address, 0);
  roboclaw2.ForwardM2(address, 0);
}

float convertTemperature(float rawTemperature) {
  float realTemperature = (1.8)*(rawTemperature - 273.15) + 32;
  return realTemperature;
}