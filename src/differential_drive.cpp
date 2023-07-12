#include <differential_drive.h>

void differentialDrive::setup(){
        V::RemoteControl remote;
    
        pinMode(remote.channel1, INPUT);
        pinMode(remote.channel2, INPUT);
}

void differentialDrive::execution(RoboClaw right, RoboClaw left){
    V::RemoteControl remote;
    V::Roboclaw roboclaw;

    remote.chan_1_read = pulseIn(remote.channel1,HIGH,30000);
    remote.chan_2_read = pulseIn(remote.channel2,HIGH,30000);

    if (remote.chan_1_read > 2000.0) remote.chan_1_read = 2000.0;
    if (remote.chan_1_read < 1000.0) remote.chan_1_read = 1000.0;

    if (remote.chan_2_read > 2000.0) remote.chan_2_read = 2000.0;
    if (remote.chan_2_read < 1000.0) remote.chan_2_read = 1000.0;

    float wheelsGap = 0.711; // m
    float wheelRadius = 0.33; // m
    float V = map(remote.chan_1_read, 1000.0, 2000.0, -1.866, 1.866); // m/s
    float W = map(remote.chan_2_read, 1000.0, 2000.0, -2.624, 2.624); // rad/sec


    float wr = (V + 0.5*wheelsGap*W)/wheelRadius;
    float wl = (V - 0.5*wheelsGap*W)/wheelRadius;
    float vr = map(wr, 0.0, 5.65, 0.0, 126.0);
    float vl = map(wl, 0.0, 5.65, 0.0, 126.0);


    if (vr > 126)
        vr = 126;
    if (vr < -126)
        vr = -126;

    if (vl > 126)
        vl = 126;
    if (vl < -126)
        vl = -126;

    motionCommands::differentialControl(right, left, vl, vr, roboclaw.address);
}

void differentialDrive::rosexecution(float x, float y, RoboClaw right, RoboClaw left){
    V::Roboclaw roboclaw;
    float wheelsGap = 0.711; // m
    float wheelRadius = 0.33; // m
    float V = map(x, -1.0, 1.0, -1.866, 1.866); // m/s
    float W = map(y, -1.5, 1.5, -2.624, 2.624); // rad/sec


    float wr = (V + 0.5*wheelsGap*W)/wheelRadius;
    float wl = (V - 0.5*wheelsGap*W)/wheelRadius;
    float vr = map(wr, 0.0, 5.65, 0.0, 126.0);
    float vl = map(wl, 0.0, 5.65, 0.0, 126.0);


    if (vr > 126)
        vr = 126;
    if (vr < -126)
        vr = -126;

    if (vl > 126)
        vl = 126;
    if (vl < -126)
        vl = -126;

    motionCommands::differentialControl(right, left, vl, vr, roboclaw.address);
}