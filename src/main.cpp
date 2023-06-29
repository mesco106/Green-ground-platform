#include "Arduino.h"
#include "arc_roboclaw/remote_control.h"
#include "Serial/SerialPort.h"
#include "arc_roboclaw/linear_actuator.h"
#include "roboclaw.h"

void setup() {
    remoteControl::setup();
    linearActuator::setup();
    roboclaw::setup();

    while(1){
        linearActuator::ActuateSolarPanels();
        remoteControl::execution(roboclaw::roboclaw1, roboclaw::roboclaw2);
    }
}

void loop(){}