#include <FEHMotor.h>
#include <FEHIO.h>
#include "simplemovement.h"

simplemovement::simplemovement() {
    this->motor1 = motor1(FEHMotor::Motor0, 9.0);
    this->motor2 = motor2(FEHMotor::Motor1, 9.0);
    this->motor3 = motor3(FEHMotor::Motor2, 9.0);
    this->motor_1_encoder = motor_1_encoder(FEHIO::P0_0);
    this->motor_2_encoder = motor_2_encoder(FEHIO::P0_1);
    this->motor_3_encoder = motor_3_encoder(FEHIO::P0_2);
}

void simplemovement::allStop() {
    this->motor1.Stop();
    this->motor2.Stop();
    this->motor3.Stop();
}

void simplemovement::moveForward(int power, int counts) {
    this->motor_1_encoder.ResetCounts();
    this->motor_2_encoder.ResetCounts();
    this->motor_3_encoder.ResetCounts();
    this->motor1.SetPercent(power);
    this->motor2.SetPercent(power);
    this->motor3.Stop();
    while(motor_1_encoder.Counts() < counts);
    this->allStop();
}

void simplemovement::moveBackward(int power, int counts) {

}

void simplemovement::moveRight(int power, int counts) {

}

void simplemovement::moveLeft(int power, int counts) {

}

void simplemovement::spinClockwise(int power, int counts) {

}

void simplemovement::spinCounterClockwise(int power, int counts) {

}
