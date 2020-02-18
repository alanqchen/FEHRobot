#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <math.h>

#define MOTOR_ANGLE_1 30.0*M_PI/180.0
#define MOTOR_ANGLE_2 150*M_PI/180.0
#define MOTOR_ANGLE_3 270*M_PI/180.0
#define MAX_RPM 236.5398195
#define R 0.11176
#define r 0.03


FEHMotor motor1(FEHMotor::Motor0, 9.0);
FEHMotor motor2(FEHMotor::Motor1, 9.0);
FEHMotor motor3(FEHMotor::Motor2, 9.0);

DigitalEncoder motor1_encoder(FEHIO::P0_0);
DigitalEncoder motor2_encoder(FEHIO::P0_1);
DigitalEncoder motor3_encoder(FEHIO::P0_2);

void allStop() {
    motor1.Stop();
    motor2.Stop();
    motor3.Stop();
}

float limitMotorPercent(float percent) {
    if (percent > 100.0) {
        return 100.0;
    } else if (percent < 0.0) {
        return 0.0;
    }
    return percent;
}

void setRadSToPercent(float motor1_RadS, float motor2_RadS, float motor3_RadS) {
    float percent1 = (((60.0/(2*M_PI))*motor1_RadS)/MAX_RPM) * 100;
    float percent2 = (((60.0/(2*M_PI))*motor2_RadS)/MAX_RPM) * 100;
    float percent3 = (((60.0/(2*M_PI))*motor3_RadS)/MAX_RPM) * 100;
    percent1 = limitMotorPercent(percent1);
    percent2 = limitMotorPercent(percent2);
    percent3 = limitMotorPercent(percent3);
    motor1.SetPercent(percent1);
    motor2.SetPercent(percent2);
    motor3.SetPercent(percent3);
}

/*
 * xVel - m/s
 * theta - rad
 * phi - rad/s
 */
void kinematics(float xVel, float yVel, float theta, float phi) {
    float angVel1 = (-1 * sin(theta+MOTOR_ANGLE_1)*cos(theta)*xVel+cos(theta+MOTOR_ANGLE_1)*cos(theta)*yVel+R*phi)/r;
    float angVel2 = (-1 * sin(theta+MOTOR_ANGLE_2)*cos(theta)*xVel+cos(theta+MOTOR_ANGLE_2)*cos(theta)*yVel+R*phi)/r;
    float angVel3 = (-1 * sin(theta+MOTOR_ANGLE_3)*cos(theta)*xVel+cos(theta+MOTOR_ANGLE_3)*cos(theta)*yVel+R*phi)/r;
    setRadSToPercent(angVel1, angVel2, angVel3);
}

// vel - 0.215-> 25% 0.85 -> 100%
// generally keep UNDER .7; .5 or under to be safe
void moveForwardBackward(float vel, float theta) {
    kinematics(0.0, vel, theta, 0.0);
}

// .7 MAX
void moveLeftRight(float vel, float theta) {
    kinematics(vel, 0.0, theta, 0.0);
}

void spinClockwise(float vel, float theta) {
    kinematics(0.0, vel, theta, 0.0);
}

// postive for counter clockwise
// 6.6 MAX, 1.65-> 25%
void spin(float phi, float theta) {
    kinematics(0.0, 0.0, theta, phi);
}

int main(void)
{

    moveForwardBackward(0.215, 0.0);
    Sleep(3.0);
    moveForwardBackward(-0.215, 0.0);
    Sleep(3.0);
    moveLeftRight(0.3, 0.0);
    Sleep(2.0);
    allStop();

    return 0;
}
