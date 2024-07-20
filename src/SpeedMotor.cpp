#include "SpeedMotor.h"

void SpeedMotor::setPower(float pwr) {
    power = pwr;
}
void SpeedMotor::setDirection(bool dir) {
    direction = dir;
}
void SpeedMotor::setPosition(long pos) {
    position = pos;
}
void SpeedMotor::setSpeed(float spd) {
    speed = spd;
}
float SpeedMotor::getPower() {
    return power;
}
long SpeedMotor::getPosition() {
    return position;
}
bool SpeedMotor::getDirection() {
    return direction;
}
float SpeedMotor::getSpeed() {
    return speed;
}
void SpeedMotor::resetPosition() {
    position = 0;
}