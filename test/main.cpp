#include "Arduino.h"
#include "SerialPort.h"
#include "Gpio.h"

void run(const String &message);

SerialPort port(115200, run);
Gpio led(LED_BUILTIN);

void setup() {
  port.setup();
  led.setup();
}

void loop() {
  port.update();
}

void run(const String &message) {
    auto linear_vel = command(message);
    auto angular_vel = argument(message, 1);
    port.send(linear_vel);
    port.send(angular_vel);
}

void move(float vx, float vy) {
  
}

void look(float angle) {
  
}

