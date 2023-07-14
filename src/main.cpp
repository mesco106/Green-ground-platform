#include "Arduino.h"
#include "remote_control.h"
#include "SerialPort.h"
#include "linear_actuator.h"
#include "differential_drive.h"
#include "gpr_riser.h"
 #include "roboclaw.h"
#include "port.h"
#include "pseudo_ackermman.h"
#include "variables.h"

void setup()
{
    // Setting communication interfaces
    Serial.begin(9600);
    roboclaw::setup();
    serial::setup();

    // Setting peripherals
    remoteControl::setup();
    steering::setup(roboclaw::roboclaw3);
    linearActuator::setup();
    gprRiser::setup();

    V::RemoteControl remoteGPIO;
    
    pinMode(remoteGPIO.auxTrigger1, INPUT);

    // int state = 0;
    //roboclaw::roboclaw3.SpeedDistanceM1(0x80,1000,0,0);
    
    while (1)
    {
        //roboclaw::roboclaw3.SpeedM1(0x80, 0);
            // roboclaw::roboclaw3.SpeedDistanceM1(0x80,1000,0,0);
            // break;
    
        
        steering::execution(roboclaw::roboclaw3, roboclaw::roboclaw1, roboclaw::roboclaw2);


        // state = digitalRead(remoteGPIO.auxTrigger1);
        // if (state == 0)
        // {
        //     linearActuator::ActuateSolarPanels();
        //     remoteControl::execution(roboclaw::roboclaw1, roboclaw::roboclaw2);
        //     gprRiser::ActuateGPRRiser();
        // }
        // else
        // {
        //     serial::update();
        // }
    }
}

void loop() {}