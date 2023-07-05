#ifndef PORT_H
#define PORT_H

#define DEBUG

#ifdef DEBUG
#define debug(x) port.send(x)
#else
#define debug(x)
#endif

#include "Arduino.h"
#include "Gpio.h"
#include "variables.h"
#include "arc_roboclaw/roboclaw.h"
#include "Serial/SerialPort.h"
#include "arc_roboclaw/motion_commands.h"

namespace serial
{
    Gpio led(13);
    void run(const String &message);
    float convertTemperature(uint16_t rawTemperature);
    SerialPort port(115200, run);

    void setup()
    {
        port.setup();
    }

    void update()
    {
        port.update();
    }

    void run(const String &message)
    {
        V::Roboclaw roboclaw;
        auto action = command(message);
        debug(action);
        if (action == "S")
        {   
            float v1 = argument(message, 1);
            float v2 = argument(message, 2);
            debug(v1);
            debug(v2);
            differentialDrive::rosexecution(v1,v2,roboclaw::roboclaw1, roboclaw::roboclaw2);
        }
        else if (action == "on")
        {
            led.on();
        }
        else if (action == "off")
        {
            led.off();
        }
        else if (action == "hi")
        {
            debug("Hi there!");
        }
        else if (action == "fo")
        {
            float speed = argument(message, 1);
            motionCommands::forward(roboclaw::roboclaw1, roboclaw::roboclaw2, speed, roboclaw.address);
            debug("Moving forwards at: " + String(speed));
        }
        else if (action == "ba")
        {
            float speed = argument(message, 1);
            motionCommands::backward(roboclaw::roboclaw1, roboclaw::roboclaw2, speed, roboclaw.address);
            debug("Moving backwards at: " + String(speed));
        }
        else if (action == "le")
        {
            float speed = argument(message, 1);
            motionCommands::left(roboclaw::roboclaw1, roboclaw::roboclaw2, speed, roboclaw.address);
            debug("Moving front at: " + String(speed));
        }
        else if (action == "ri")
        {
            float speed = argument(message, 1);
            motionCommands::right(roboclaw::roboclaw1, roboclaw::roboclaw2, speed, roboclaw.address);
            debug("Moving rear at: " + String(speed));
        }
        else if (action == "s")
        {
            motionCommands::stop(roboclaw::roboclaw1);
            motionCommands::stop(roboclaw::roboclaw2);
            debug("motors stopped");
        }
        else if (action == "v4")
        {
            float v1 = argument(message, 1);
            float v2 = argument(message, 2);
            float v3 = argument(message, 3);
            float v4 = argument(message, 4);
            motionCommands::individualControl(roboclaw::roboclaw1, roboclaw::roboclaw2, v1, v2, v3, v4, roboclaw.address);
            debug("v1: " + String(v1));
            debug("v2: " + String(v2));
            debug("v3: " + String(v3));
            debug("v4: " + String(v4));
        }
        else if (action == "dc")
        {
            float vl = argument(message, 1);
            float vr = argument(message, 2);
            motionCommands::differentialControl(roboclaw::roboclaw1, roboclaw::roboclaw2, vl, vr, roboclaw.address);
            debug("vl: " + String(vl));
            debug("vr: " + String(vr));
        }
        else if (action == "se1")
        {
            int32_t value = argument(message, 1);
            roboclaw::roboclaw1.SetEncM1(roboclaw.address, value);
            debug("Encoder 1 setup to: " + String(value));
        }
        else if (action == "se2")
        {
            int32_t value = argument(message, 1);
            roboclaw::roboclaw1.SetEncM2(roboclaw.address, value);
            debug("Encoder 2 setup to: " + String(value));
        }
        else if (action == "se3")
        {
            int32_t value = argument(message, 1);
            roboclaw::roboclaw2.SetEncM1(roboclaw.address, value);
            debug("Encoder 3 setup to: " + String(value));
        }
        else if (action == "se4")
        {
            int32_t value = argument(message, 1);
            roboclaw::roboclaw2.SetEncM2(roboclaw.address, value);
            debug("Encoder 4 setup to: " + String(value));
        }
        else if (action == "rese")
        {
            roboclaw::roboclaw1.ResetEncoders(roboclaw.address);
            roboclaw::roboclaw2.ResetEncoders(roboclaw.address);
            debug("Encoders reseted!");
        }
        else if (action == "re1")
        {
            debug("Encoder 1: " + String(roboclaw::roboclaw1.ReadEncM1(roboclaw.address)));
        }
        else if (action == "re2")
        {
            debug("Encoder 2: " + String(roboclaw::roboclaw1.ReadEncM2(roboclaw.address)));
        }
        else if (action == "re3")
        {
            debug("Encoder 3: " + String(roboclaw::roboclaw2.ReadEncM1(roboclaw.address)));
        }
        else if (action == "re4")
        {
            debug("Encoder 4: " + String(roboclaw::roboclaw2.ReadEncM2(roboclaw.address)));
        }
        else if (action == "re")
        {
            debug("Encoders : " + String(roboclaw::roboclaw2.ReadEncM2(roboclaw.address)) + ' ' + String(roboclaw::roboclaw2.ReadEncM1(roboclaw.address)) + ' ' + String(roboclaw::roboclaw2.ReadEncM1(roboclaw.address)) + ' ' + String(roboclaw::roboclaw2.ReadEncM2(roboclaw.address)));
        }
        else if (action == "t")
        {
            uint16_t temp1;
            uint16_t temp2;
            uint16_t temp3;
            uint16_t temp4;
            roboclaw::roboclaw1.ReadTemp(roboclaw.address, temp1);
            roboclaw::roboclaw1.ReadTemp(roboclaw.address, temp1);
            roboclaw::roboclaw1.ReadTemp2(roboclaw.address, temp2);
            roboclaw::roboclaw2.ReadTemp(roboclaw.address, temp3);
            roboclaw::roboclaw2.ReadTemp2(roboclaw.address, temp4);
            debug("Temperature " + String(convertTemperature(temp1)) + ' ' + String(convertTemperature(temp2)) + ' ' + String(convertTemperature(temp3)) + ' ' + String(convertTemperature(temp4)));
        }
        else if (action == "b")
        {
            debug("Battery " + String(roboclaw::roboclaw1.ReadMainBatteryVoltage(roboclaw.address)) + ' ' + String(roboclaw::roboclaw2.ReadMainBatteryVoltage(roboclaw.address)));
        }
        else if (action == "v")
        {
            char version[32];
            if (roboclaw::roboclaw1.ReadVersion(roboclaw.address, version))
                debug(String("Version ") + version);
        }
        else if (action == "ss")
        {
            debug("Speed " + String(roboclaw::roboclaw1.ReadSpeedM1(roboclaw.address)) + ' ' + String(roboclaw::roboclaw1.ReadSpeedM2(roboclaw.address)) + ' ' + String(roboclaw::roboclaw2.ReadSpeedM1(roboclaw.address)) + ' ' + String(roboclaw::roboclaw2.ReadSpeedM2(roboclaw.address)));
        }
        else
        {
            debug("Ops, '" + message + "' command not found!");
        }
    }

    float convertTemperature(uint16_t rawTemperature) {
        float realTemperature = (1.8)*(rawTemperature - 273.15) + 32;
    return realTemperature;
    }
}

#endif