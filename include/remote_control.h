#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "Arduino.h"
#include <RoboClaw.h>

namespace remoteControl {
    void setup();
    void execution(RoboClaw front, RoboClaw rear);
    int state(const uint8_t &pin);

}

#endif