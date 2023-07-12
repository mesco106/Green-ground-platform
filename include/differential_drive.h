#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "variables.h"
#include "motion_commands.h"
#
#include <RoboClaw.h>

namespace differentialDrive {
    void setup();
    void execution(RoboClaw right, RoboClaw left);
    void rosexecution(float x, float y, RoboClaw right, RoboClaw left);
}

#endif