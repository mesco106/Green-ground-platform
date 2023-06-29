#include "arc_roboclaw/remote_control.h"
#include "variables.h"


void remoteControl::setup(){
    V::RemoteControl remote;
    
    pinMode(remote.channel1, INPUT);
    pinMode(remote.channel2, INPUT);
}

void remoteControl::execution(RoboClaw right, RoboClaw left){
    V::RemoteControl remote;
    V::Set set;

    remote.chan_1_read = pulseIn(remote.channel1,HIGH,36000);
    remote.chan_2_read = pulseIn(remote.channel2,HIGH,36000);
    
    if (remote.chan_1_read > 2000) remote.chan_1_read = 2000;
    if (remote.chan_1_read < 1000) remote.chan_1_read = 1000;

    if (remote.chan_2_read > 2000) remote.chan_2_read = 2000;
    if (remote.chan_2_read < 1000) remote.chan_2_read = 1000;

    if (remote.chan_1_read > 1600) {
        set.motor_1_spd = map(remote.chan_1_read, 1550,2000,0,126);
        set.motor_2_spd = map(remote.chan_1_read, 1550,2000,0,126);

        right.ForwardM1(address, set.motor_1_spd);
        right.ForwardM2(address, set.motor_1_spd);

        left.ForwardM1(address, set.motor_2_spd);
        left.ForwardM2(address, set.motor_2_spd);

    } else if (remote.chan_1_read < 1400){
        set.motor_1_spd = map(remote.chan_1_read, 1450,1000,0,126);
        set.motor_2_spd = map(remote.chan_1_read, 1450,1000,0,126);

        right.BackwardM1(address,set.motor_1_spd);
        right.BackwardM2(address,set.motor_1_spd);

        left.BackwardM1(address,set.motor_2_spd);
        left.BackwardM2(address,set.motor_2_spd);

    } else if (remote.chan_2_read > 1600){

        set.motor_2_spd = map(remote.chan_2_read, 1550,2000,0,126);
        set.motor_1_spd = map(remote.chan_2_read, 1550,2000,0,126);
        right.ForwardM1(address, set.motor_2_spd);
        right.ForwardM2(address, set.motor_2_spd);

        left.BackwardM1(address,set.motor_2_spd);
        left.BackwardM2(address,set.motor_2_spd);

    } else if (remote.chan_2_read < 1400){
        //TURN
        set.motor_2_spd = map(remote.chan_2_read, 1450,1000,0,126);
        set.motor_1_spd = map(remote.chan_2_read, 1450,1000,0,126);
        right.BackwardM1(address, set.motor_2_spd);
        right.BackwardM2(address, set.motor_2_spd);

        left.ForwardM1(address,set.motor_2_spd);
        left.ForwardM2(address,set.motor_2_spd);

    } else if ((remote.chan_1_read > 1450) && (remote.chan_1_read < 1550)){
        right.ForwardM1(address,set.motor_stop);
        right.ForwardM2(address,set.motor_stop);

        left.ForwardM1(address,set.motor_stop);
        left.ForwardM2(address,set.motor_stop);
    }
}