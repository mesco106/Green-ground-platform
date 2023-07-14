#include "linear_actuator.h"
#include "Arduino.h"
#include "variables.h"
#include "Gpio.h"

void linearActuator::setup(){
    V::LinearActuator linearActuator;
    Gpio led(linearActuator.warningLED);
    led.setup();

    pinMode(linearActuator.enA, OUTPUT);
    pinMode(linearActuator.in1, OUTPUT);
    pinMode(linearActuator.in2, OUTPUT);

    digitalWrite(linearActuator.in1, HIGH);
    digitalWrite(linearActuator.in2, LOW);
    analogWrite(linearActuator.enA, 255);

    analogWrite(linearActuator.enA, 0);
}

void linearActuator::ActuateSolarPanels(){
    V::LinearActuator linearActuator;
    Gpio led(linearActuator.warningLED);
    int input = pulseIn(linearActuator.trigger, HIGH, 20000);

    if (input > 1700){
        digitalWrite(linearActuator.in1, HIGH);
        digitalWrite(linearActuator.in2, LOW);
        analogWrite(linearActuator.enA, 255);
        //led.blink(500);
        led.off();
    } else if (input < 1300 && input > 800){
        digitalWrite(linearActuator.in1, LOW);
        digitalWrite(linearActuator.in2, HIGH);
        analogWrite(linearActuator.enA, 255);
        //led.blink(500);
        led.off();
    } else {
        analogWrite(linearActuator.enA, 0);
        led.on();
    }
}

void linearActuator::RosActuateSolarPanels(bool node){
    V::LinearActuator linearActuator;
    Gpio led(linearActuator.warningLED);

    if (node == true){
        digitalWrite(linearActuator.in1, HIGH);
        digitalWrite(linearActuator.in2, LOW);
        analogWrite(linearActuator.enA, 255);
        led.on();
    } else if (node == false) {
        digitalWrite(linearActuator.in1, LOW);
        digitalWrite(linearActuator.in2, HIGH);
        analogWrite(linearActuator.enA, 255);
        led.off();
    } 
}
