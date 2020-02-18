#ifndef SIMPLEMOVEMENT_H
#define SIMPLEMOVEMENT_H

#include <FEHMotor.h>
#include <FEHIO.h>
/*
 * m2-----m1
 *   \    /
 *     m3
 */

class simplemovement
{
public:
    simplemovement(FEHMotor m1, FEHMotor m2, FEHMotor m3, DigitalEncoder e1, DigitalEncoder e2, DigitalEncoder e3);
    void allStop();
    void moveForward(int power, int counts);
    void moveBackward(int power, int counts);
    void moveRight(int power, int counts);
    void moveLeft(int power, int counts);
    void spinClockwise(int power, int counts);
    void spinCounterClockwise(int power, int counts);
private:
    FEHMotor motor1;
    FEHMotor motor2;
    FEHMotor motor3;
    DigitalEncoder motor_encoder_1;
    DigitalEncoder motor_encoder_2;
    DigitalEncoder motor_encoder_3;
};

#endif // SIMPLEMOVEMENT_H
