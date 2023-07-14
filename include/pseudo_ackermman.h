#ifndef PSEUDO_ACKERMMAN_H
#define PSEUDO_ACKERMMAN_H

#include "variables.h"
#include <RoboClaw.h>
#include "motion_commands.h"

namespace steering {

    void setup(RoboClaw steering);
    void execution(RoboClaw steering, RoboClaw left, RoboClaw right);
    void rosexecution(float x, float y, RoboClaw steering, RoboClaw left, RoboClaw right);
}

#endif 