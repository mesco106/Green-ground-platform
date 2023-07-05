#include "Arduino.h"
#include "arc_roboclaw/remote_control.h"
#include "Serial/SerialPort.h"
#include "arc_roboclaw/linear_actuator.h"
#include "arc_roboclaw/differential_drive.h"
#include "arc_roboclaw/roboclaw.h"
#include "port.h"

SerialPort port(115200, serial::run);

void setup() {
    remoteControl::setup();
    linearActuator::setup();
    roboclaw::setup();
    port.setup();
    //serial::setup();

    while(1){
        port.update();
        //linearActuator::ActuateSolarPanels();
        //serial::update();
        //remoteControl::execution(roboclaw::roboclaw1, roboclaw::roboclaw2);
        //differentialDrive::execution(roboclaw::roboclaw1, roboclaw::roboclaw2);
    }
}

void loop(){}