#ifndef VARIABLES_H
#define VARIABLES_H

#include "Arduino.h"

namespace V {
    struct Roboclaw
    {

    };

    struct RemoteControl
    {   
        uint8_t channel1 = 22;
        uint8_t channel2 = 23;
        uint16_t chan_1_read;
        uint16_t chan_2_read;
    };

    struct Set
    {
        uint8_t motor_1_spd = 0;
        uint8_t motor_2_spd = 0;
        uint8_t motor_stop = 0;
    };

    struct LinearActuator
    {
        uint8_t enA = 37;
        uint8_t in1 = 41;
        uint8_t in2 = 40;
        uint8_t trigger = 19;
    };

}
#endif