#include "arc_roboclaw/linear_actuator.h"
#include "Arduino.h"
#include "variables.h"
#include "Gpio.h"

Gpio led(13);

void linearActuator::setup(){
    V::LinearActuator linearActuator;
    
    led.setup();

    pinMode(linearActuator.enA, OUTPUT);
    pinMode(linearActuator.in1, OUTPUT);
    pinMode(linearActuator.in2, OUTPUT);

    analogWrite(linearActuator.enA, 255);
}

void linearActuator::ActuateSolarPanels(){
    V::LinearActuator linearActuator;
    int input = pulseIn(linearActuator.trigger, HIGH, 20000);
    
    Serial.println(input);
    if (input > 1500){
        digitalWrite(linearActuator.in1, LOW);
        digitalWrite(linearActuator.in2, HIGH);
        led.on();
    } else {
        digitalWrite(linearActuator.in1, HIGH);
        digitalWrite(linearActuator.in2, LOW);
        led.off();
    }    
}
