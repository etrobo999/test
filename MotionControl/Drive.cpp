#include "Drive.h"
#include <algorithm>

Drive::Drive(ev3api::Motor& leftWheel, ev3api::Motor& rightWheel)
    : mLeftWheel(leftWheel), mRightWheel(rightWheel), mLeftBaseSpeed(80.0), mRightBaseSpeed(80.0) {
}

void Drive::setBaseSpeed(double leftSpeed, double rightSpeed) {
    mLeftBaseSpeed = leftSpeed;
    mRightBaseSpeed = rightSpeed;
}

void Drive::setMotorSpeeds(double leftSpeed, double rightSpeed) {
    leftSpeed = std::max(std::min(leftSpeed, 100.0), -100.0);
    rightSpeed = std::max(std::min(rightSpeed, 100.0), -100.0);

    mLeftWheel.setPWM(static_cast<int>(leftSpeed));
    mRightWheel.setPWM(static_cast<int>(rightSpeed));
}