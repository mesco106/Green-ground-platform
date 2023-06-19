
#ifndef STATE_H
#define STATE_H

#include "SerialPort.h"

  enum Mode {
    manualMode = 0,
    semiAutonomousMode = 1,
    rosMode = 2
  };

namespace N{

  uint8_t state(const int p_trigger) {
    Mode mode;
    int channel5 = pulseIn(p_trigger, HIGH, 36000);

    if (channel5 > 1600) 
      mode = rosMode;
    else if (channel5 >= 1300 && channel5 <= 1600)
      mode = semiAutonomousMode;
    else 
      mode = manualMode;
    

    return mode;
  }
}

#endif