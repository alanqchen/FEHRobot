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
    simplemovement();
    void allStop();
    void moveForward(int power, int counts);
    void moveBackward(int power, int counts);
    void moveRight(int power, int counts);
    void moveLeft(int power, int counts);
    void spinClockwise(int power, int counts);
    void spinCounterClockwise(int power, int counts);
private:
    FEHMotor::FEHMotor motor1;
    FEHMotor::FEHMotor motor2;
    FEHMotor::FEHMotor motor3;
};

#endif // SIMPLEMOVEMENT_H
