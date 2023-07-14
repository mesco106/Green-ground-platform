#ifndef VARIABLES_H
#define VARIABLES_H

#include "Arduino.h"

/*RESERVED PINS FOR SERIAL COMUNICATIOM
    PIN 21 RX5
    PIN 20 TX5
    PIN 17 TX4
    PIN 16 RX4
    PIN 15 RX3
    PIN 14 TX3
*/
namespace V {
    struct Roboclaw
    {
        const int address = 0x80;
        int position = 0;
    };

    struct RemoteControl
    {   
        const uint8_t warningLED = /*PIN*/30;
        const uint8_t channel1 = /*PIN*/7;
        const uint8_t channel2 = /*PIN*/8;
        const uint8_t auxTrigger1 = /*PIN*/9;
        const uint8_t auxTrigger2 = /*PIN*/10;
        float chan_1_read;
        float chan_2_read;
    };

    struct LinearActuator
    {
        const uint8_t warningLED = /*PIN*/29;
        const uint8_t enA = /*PIN*/19;
        const uint8_t in1 = /*PIN*/23;
        const uint8_t in2 = /*PIN*/22;
        const uint8_t trigger = /*PIN*/11;
    };

    struct LevelGPR {
        const uint8_t warningLED = /*PIN*/31;

        const uint8_t enA = /*PIN*/33;
        const uint8_t enB = /*PIN*/36;
        const uint8_t enC = /*PIN*/37;

        const uint8_t in1A = /*PIN*/35;
        const uint8_t in2A = /*PIN*/34;
        const uint8_t in1B = /*PIN*/41;
        const uint8_t in2B = /*PIN*/40;
        const uint8_t in1C = /*PIN*/39;
        const uint8_t in2C = /*PIN*/38;

        const uint8_t trigger = /*PIN*/12;

    };

    struct Set
    {
        int motor_1_spd = 0;
        int motor_2_spd = 0;
        uint8_t motor_stop = 0;
    };
}
#endif