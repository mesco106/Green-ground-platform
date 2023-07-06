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
    };

    struct RemoteControl
    {   
        uint8_t channel1 = /*PIN*/22;
        uint8_t channel2 = /*PIN*/23;
        float chan_1_read;
        float chan_2_read;
    };

    struct Set
    {
        uint8_t motor_1_spd = 0;
        uint8_t motor_2_spd = 0;
        uint8_t motor_stop = 0;
    };

    struct LinearActuator
    {
        uint8_t enA = /*PIN*/37;
        uint8_t in1 = /*PIN*/41;
        uint8_t in2 = /*PIN*/40;
        uint8_t trigger = /*PIN*/19;
    };

    struct LevelGPR {
        uint8_t enA = /*PIN*/36;
        uint8_t in1 = /*PIN*/35;
        uint8_t in2 = /*PIN*/34;
        uint8_t trigger = /*PIN*/37;
    };

}
#endif