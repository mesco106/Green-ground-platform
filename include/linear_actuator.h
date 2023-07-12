#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include "Arduino.h"
#include "variables.h"
#include "Gpio.h"

namespace linearActuator{
    void setup();
    void ActuateSolarPanels();
    void RosActuateSolarPanels(bool node);
}

#endif