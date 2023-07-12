#include "remote_control.h"
#include "variables.h"

void remoteControl::setup(){
    V::RemoteControl remote;
    
    pinMode(remote.channel1, INPUT);
    pinMode(remote.channel2, INPUT);
    pinMode(remote.auxTrigger1, INPUT);
    pinMode(remote.auxTrigger2, INPUT);
    pinMode(remote.warningLED, OUTPUT);

}

void remoteControl::execution(RoboClaw right, RoboClaw left){
    V::RemoteControl remote;
    V::Roboclaw roboclaw;
    V::Set set;

    remote.chan_1_read = pulseIn(remote.channel1,HIGH,36000);
    remote.chan_2_read = pulseIn(remote.channel2,HIGH,36000);
    
    if (remote.chan_1_read < 800 && remote.chan_2_read < 800) {
        digitalWrite(remote.warningLED, LOW);
        remote.chan_1_read = 1500;
        remote.chan_2_read = 1500;
    } else {
        digitalWrite(remote.warningLED, HIGH);
    }

    if (remote.chan_1_read > 2000) remote.chan_1_read = 2000;
    if (remote.chan_1_read < 1000) remote.chan_1_read = 1000;

    if (remote.chan_2_read > 2000) remote.chan_2_read = 2000;
    if (remote.chan_2_read < 1000) remote.chan_2_read = 1000;

    if (remote.chan_1_read > 1600) {
        set.motor_1_spd = map(remote.chan_1_read, 1550,2000,0,126);
        set.motor_2_spd = map(remote.chan_1_read, 1550,2000,0,126);

        right.ForwardM1(roboclaw.address, set.motor_1_spd);
        right.ForwardM2(roboclaw.address, set.motor_1_spd);

        left.ForwardM1(roboclaw.address, set.motor_2_spd);
        left.ForwardM2(roboclaw.address, set.motor_2_spd);

    } else if (remote.chan_1_read < 1400){
        set.motor_1_spd = map(remote.chan_1_read, 1450,1000,0,126);
        set.motor_2_spd = map(remote.chan_1_read, 1450,1000,0,126);

        right.BackwardM1(roboclaw.address,set.motor_1_spd);
        right.BackwardM2(roboclaw.address,set.motor_1_spd);

        left.BackwardM1(roboclaw.address,set.motor_2_spd);
        left.BackwardM2(roboclaw.address,set.motor_2_spd);

    } else if (remote.chan_2_read > 1600){

        set.motor_2_spd = map(remote.chan_2_read, 1550,2000,0,126);
        set.motor_1_spd = map(remote.chan_2_read, 1550,2000,0,126);
        right.ForwardM1(roboclaw.address, set.motor_2_spd);
        right.ForwardM2(roboclaw.address, set.motor_2_spd);

        left.BackwardM1(roboclaw.address,set.motor_2_spd);
        left.BackwardM2(roboclaw.address,set.motor_2_spd);

    } else if (remote.chan_2_read < 1400){
        //TURN
        set.motor_2_spd = map(remote.chan_2_read, 1450,1000,0,126);
        set.motor_1_spd = map(remote.chan_2_read, 1450,1000,0,126);
        right.BackwardM1(roboclaw.address, set.motor_2_spd);
        right.BackwardM2(roboclaw.address, set.motor_2_spd);

        left.ForwardM1(roboclaw.address,set.motor_2_spd);
        left.ForwardM2(roboclaw.address,set.motor_2_spd);

    } else if ((remote.chan_1_read > 1450) && (remote.chan_1_read < 1550)){
        right.ForwardM1(roboclaw.address,set.motor_stop);
        right.ForwardM2(roboclaw.address,set.motor_stop);

        left.ForwardM1(roboclaw.address,set.motor_stop);
        left.ForwardM2(roboclaw.address,set.motor_stop);
    }
}

int remoteControl::state(const uint8_t &pin){
    uint16_t triggerReading = digitalRead(pin);
    if (triggerReading > 1900)
        return 2;
    if (triggerReading < 1100)
        return 1;
    if (triggerReading >= 1100 && triggerReading <= 1900)
        return 0;
    return 1;
}