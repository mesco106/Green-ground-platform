#include "pseudo_ackermman.h"

int pass = 0;

void steering::setup(RoboClaw steering)
{
    V::Roboclaw Steering;
    steering.ResetEncoders(Steering.address);

    pinMode(Steering.rotaryEncoder ,INPUT);
}

void steering::execution(RoboClaw steering, RoboClaw left, RoboClaw right)
{
    V::Roboclaw Steering;
    V::RemoteControl remote;
    V::Roboclaw roboclaw;
    V::Set set;
    int speedLimit = 10;

    Steering.position = steering.ReadEncM1(Steering.address);

    remote.chan_1_read = pulseIn(remote.channel1, HIGH, 30000);
    remote.chan_2_read = pulseIn(remote.channel2, HIGH, 30000);
    Steering.readPosition = pulseIn(Steering.rotaryEncoder, HIGH, 30000);
    Serial.println(Steering.readPosition);
    // Serial.print("  |  ");
    // Serial.println(remote.chan_2_read);


    //float *flag1 = &remote.chan_1_read;
    //float *flag2 = &remote.chan_2_read;

    if (remote.chan_1_read < 800 && remote.chan_2_read < 800)
    {
        digitalWrite(remote.warningLED, LOW);
        remote.chan_1_read = 1500;
        remote.chan_2_read = 1500;
    }
    else
    {
        digitalWrite(remote.warningLED, HIGH);
    }

    if (remote.chan_1_read > 2000.0)
        remote.chan_1_read = 2000.0;
    if (remote.chan_1_read < 1000.0)
        remote.chan_1_read = 1000.0;

    if (remote.chan_2_read > 2000.0)
        remote.chan_2_read = 2000.0;
    if (remote.chan_2_read < 1000.0)
        remote.chan_2_read = 1000.0;

    if (remote.chan_1_read > 1600)
    {
        set.motor_1_spd = map(remote.chan_1_read, 1550, 2000, 0, 126);
        set.motor_2_spd = map(remote.chan_2_read, 1000, 2000, -speedLimit, speedLimit);
        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        motionCommands::forward(left, right, set.motor_1_spd, roboclaw.address);
        pass = 0;
    }
    else if (remote.chan_1_read < 1400)
    {
        set.motor_1_spd = map(remote.chan_1_read, 1450, 1000, 0, 126);
        set.motor_2_spd = map(remote.chan_2_read, 1000, 2000, -speedLimit, speedLimit);
        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        motionCommands::backward(left, right, set.motor_1_spd, roboclaw.address);
        pass = 0;
    }
    else if (remote.chan_2_read > 1600)
    {
        // TURN
        set.motor_1_spd = map(remote.chan_1_read, 1550, 2000, 0, 126);
        set.motor_2_spd = map(remote.chan_2_read, 1000, 2000, -speedLimit, speedLimit);

        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        pass = 0;
    }
    else if (remote.chan_2_read < 1400)
    {
        // TURN
        set.motor_1_spd = map(remote.chan_2_read, 1450, 1000, 0, 126);
        set.motor_2_spd = map(remote.chan_2_read, 1000, 2000, -speedLimit, -speedLimit);

        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        pass = 0;
    }
    else if ((remote.chan_1_read >= 1400) && (remote.chan_1_read <= 1600))
    {
        pass++;

        steering.SpeedM1(Steering.address, 0);
        right.ForwardM1(roboclaw.address, set.motor_stop);
        right.ForwardM2(roboclaw.address, set.motor_stop);
        left.ForwardM1(roboclaw.address, set.motor_stop);
        left.ForwardM2(roboclaw.address, set.motor_stop);
        

        // //steering.SpeedDistanceM1(0x80, 1000, 0, 0);
        while (pass == 1) {
                steering.SpeedDistanceM1(0x80,0,0,0);
                break;
        }
    }


}

void steering::rosexecution(float x, float y, RoboClaw steering, RoboClaw left, RoboClaw right){
    V::Roboclaw Steering;
    V::Set set;
    int speedLimit = 10;
    Serial.print(x);
    Serial.print("  |  ");
    Serial.println(y);


    if (x > 0.1) {
        set.motor_1_spd = map(x, 0, 1.5, 0, 126);
        set.motor_2_spd = map(y, 1.0, -1.0, -speedLimit, speedLimit);
        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        motionCommands::forward(left, right, set.motor_1_spd, Steering.address);
        pass = 0;

    } else if (x < -0.1){
        set.motor_1_spd = map(x, 0, -1.5, 0, 126);
        set.motor_2_spd = map(y, 1.0, -1.0, -speedLimit, speedLimit);
        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        motionCommands::backward(left, right, set.motor_1_spd, Steering.address);
        pass = 0;
    } else if (y > 0.1){
        set.motor_2_spd = map(y, 1.0, -1.0, -speedLimit, speedLimit);
        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        pass = 0;
    } else if (y < -0.1){
        set.motor_2_spd = map(y, 1.0, -1.0, -speedLimit, speedLimit);
        motionCommands::steering(steering, set.motor_2_spd, Steering.address);
        pass = 0;
    } else if (y >= -0.1 && y <= 0.1) {
        pass++;
        steering.SpeedM1(Steering.address, 0);
        motionCommands::forward(left, right, set.motor_stop, Steering.address);

        while (pass == 1) {
            steering.SpeedDistanceM1(0x80,0,0,0);
            break;
        }
    }
    // motionCommands::forwardbackward(left, right, V, Steering.address);
    // motionCommands::steering(steering, W, Steering.address);

}