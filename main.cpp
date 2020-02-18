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
#define ENCODER_RES 318.0
#define DELTA_T 0.1
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

float inchestoMeters(float num) {
    return num/39.37;
}

float countsToRadDisp(int new, int old) {
    int difference = new-old;
    //float degDif = (float)difference * 360.0/318.0;
    //return degDif * (M_PI / 180.0);
    return (float)difference*(2.0*M_PI)/ENCODER_RES;
}

// Give in inches (for RPS)
void PIDMoveTo(float x, float y) {
    float xMeters = inchestoMeters(x);
    float yMeters = inchestoMeters(y);
    int countNew1 = 0;
    int countNew2 = 0;
    int countNew3 = 0;
    int countOld1 = 0;
    int countOld2 = 0;
    int countOld3 = 0;
    float displacement1 = 0.0;
    float displacement2 = 0.0;
    float displacement3 = 0.0;
    float phiVel1 = 0.0;
    float phiVel2 = 0.0;
    float phiVel3 = 0.0;
    float phi1 = 0.0;
    float phi2 = 0.0;
    float phi3 = 0.0;
    float motorSpeed1 = 0.0; 
    float motorSpeed2 = 0.0; 
    float motorSpeed3 = 0.0; 
    float errorCurr1 = 0.0;
    float errorCurr2 = 0.0;
    float errorCurr3 = 0.0;
    float errorTotal1 = 0.0;
    float errorTotal2 = 0.0;
    float errorTotal3 = 0.0;
    float Kp = 1.0;
    float Ki = 0.0;
    float Kd = 0.0;
    float pidMarginError = 0.1; // in inches
    
    bool setup = true;
    
    while (sqrt(pow(RPS.X(),2)+pow(RPS.Y(),2)) > pidMarginError) {
        countNew1 = motor1_encoder.Counts();
        countNew2 = motor2_encoder.Counts();
        countNew3 = motor3_encoder.Counts();
        if(setup) {
            displacement1 = countsToRadDisp(countNew1, 0);
            displacement2 = countsToRadDisp(countNew2, 0);
            displacement3 = countsToRadDisp(countNew3, 0);
            setup = false;
        } else {
            displacement1 = countsToRadDisp(countNew1, countOld1);
            displacement2 = countsToRadDisp(countNew2, countOld1);
            displacement3 = countsToRadDisp(countNew3, countOld1);
        }
        phi1 += displacement1;
        phi2 += displacement2;
        phi3 += displacement3;
        phiVel1 = displacement1/DELTA_T;
        phiVel2 = displacement2/DELTA_T;
        phiVel3 = displacement3/DELTA_T;
        
        // TODO: IMPLEMENT TRAJECTORY PROFILE PARSER
        // PROFILE -> PARSE USING KINEMATIC RELATIONSHIPS -> REFERENCE MOTION -> INTEGRATION -> REFERENCE POSITION
    }
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
