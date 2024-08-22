#pragma once

#include "Motor.h"

class Drive {
public:
    Drive(ev3api::Motor& leftWheel, ev3api::Motor& rightWheel);

    void setBaseSpeed(double leftSpeed, double rightSpeed);
    void setMotorSpeeds(double leftSpeed, double rightSpeed);

protected:
    ev3api::Motor& mLeftWheel;
    ev3api::Motor& mRightWheel;
    double mLeftBaseSpeed;
    double mRightBaseSpeed;
};