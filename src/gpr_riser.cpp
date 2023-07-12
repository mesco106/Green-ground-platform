#include "gpr_riser.h"

void gprRiser::setup()
{
    V::LevelGPR gprRiser;
    Gpio led(gprRiser.warningLED);
    led.setup();

    pinMode(gprRiser.enA, OUTPUT);
    pinMode(gprRiser.in1A, OUTPUT);
    pinMode(gprRiser.in2A, OUTPUT);

    analogWrite(gprRiser.enA, 0);
}

void gprRiser::ActuateGPRRiser()
{
    V::LevelGPR gprRiser;
    Gpio led(gprRiser.warningLED);

    int input = pulseIn(gprRiser.trigger, HIGH, 20000);
    if (input > 1900)
    {
        digitalWrite(gprRiser.in1A, LOW);
        digitalWrite(gprRiser.in2A, HIGH);

        digitalWrite(gprRiser.in1B, LOW);
        digitalWrite(gprRiser.in2B, HIGH);

        digitalWrite(gprRiser.in1C, LOW);
        digitalWrite(gprRiser.in2C, HIGH);

        analogWrite(gprRiser.enA, 255);
        analogWrite(gprRiser.enB, 255);
        analogWrite(gprRiser.enC, 255);

        led.off();
    }
    else if (input < 1100)
    {
        digitalWrite(gprRiser.in1A, HIGH);
        digitalWrite(gprRiser.in2A, LOW);

        digitalWrite(gprRiser.in1B, HIGH);
        digitalWrite(gprRiser.in2B, LOW);

        digitalWrite(gprRiser.in1C, HIGH);
        digitalWrite(gprRiser.in2C, LOW);

        analogWrite(gprRiser.enA, 255);
        analogWrite(gprRiser.enB, 255);
        analogWrite(gprRiser.enC, 255);

        led.off();
    }
    else
    {
        analogWrite(gprRiser.enA, 0);
        analogWrite(gprRiser.enB, 0);
        analogWrite(gprRiser.enC, 0);
        led.on();
    }
}