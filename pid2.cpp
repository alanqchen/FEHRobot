#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <cmath>

/* Alan Chen - FEH 2020
 * Mathematical derivation from:
 * Baede, T. A. (2006). Motion control of an omnidirectional mobile robot. (DCT rapporten; Vol. 2006.084).
 *   Eindhoven: Technische Universiteit Eindhoven.
 * With help from "PID without a PHD"
 */

#define L 0.09906 // distance between body center and wheel center (approx 3.9 inches to meters)
#define r 0.03 // wheel radius (60 mm - > meter -> /2)
#define stopAngle 0.104719755
#define stopDistance 0.001
#define sqrt3 1.7320508
#define sqrt32 0.8660254037
#define pi2 6.28318530717
#define P 5.0 // Proportional constant
#define I 0.0005 // Integral constant
#define Vmax 1.0 // (m/s)
#define omegamax 50.0 // (rad/s)
// Global vars so don't have to keep passing in a thousand variables to every function
double Vx;
double Vy;
double Vxm;
double Vym;
double Vxw;
double Vyw;
double VxAbs;
double VyAbs;
double VxmTarget; // velocities relative to chasis
double VymTarget;
double VxwTarget; // velocities relative to course
double VywTarget;
double omegap; // Basically angular velocity
double Vback; // Wheel velocities
double Vleft;
double Vright;
double VbackTarget;
double VleftTarget;
double VrightTarget;
double omegapL;
double omegapLVxm2;
double sqrtVym2;
double Vxm2;
double Vback3;
double Vleft3;
double Vright3;
double VxTarget;
double VyTarget;
double omegapTarget;
double x = 0;
double y = 0;
double xm = 0;
double ym = 0;
double xOffset = 0;
double yOffset = 0;
double xTarget = 0;
double yTarget = 0;
double xw = 0;
double yw = 0;
double xError;
double yError;
double xErrorI;
double yErrorI;
double theta = 0;
double thetaError;
double thetaErrorI;
double thetaTarget = 0;
double timeElapsed = 0;
double phi;
double scale;



bool isStopPose() {
    return (std::abs(xError) < stopDistance)
     && (std::abs(yError) < stopDistance)
     && (std::abs(thetaError) < stopAngle);
}

// convert degrees from RPS to radian
double degree2radian(double degree) {
    return degree * M_PI/180.0;
}

// Ensures radian is in the range [0, 2pi]
double normalizeRadian(double radian) {
    radian = fmod(radian, pi2);
    if (radian < 0) radian += pi2;
    return radian;
}

void inverseKinematicsMobile() {
    omegapL = omegap * L;
    sqrtVym2 = sqrt32 * Vym;
    Vxm2 = Vxm / 2.0;
    omegapLVxm2 = omegapL - Vxm2;
    Vleft  = omegapLVxm2 - sqrtVym2;
    Vback  = omegapL + Vxm;
    Vright = omegapLVxm2 + sqrtVym2;
}

// Calculates robot velocity in reference to the chasis (no angle)
void forwardKinematicsMobile() {
    Vleft3 = Vleft / 3.0;
    Vback3 = Vback / 3.0;
    Vright3 = Vright / 3.0;
    Vxm = (2 * Vback3) - Vleft3 - Vright3;
    Vym = (sqrt3 * Vright3) - (sqrt3 * Vleft3);
    omegap = (Vleft3 + Vback3 + Vright3) / L;
}

// Calculates robot velocity in reference to the course (has angle)
void forwardKinematicsWorld() {
    forwardKinematicsMobile();
    Vxw = (std::cos(theta) * Vxm) - (std::sin(theta) * Vym);
    Vyw = (std::cos(theta) * Vym) + (std::sin(theta) * Vxm);
}

// Calculations for converting encoders to wheel speeds
void odometryCalculations(DigitalEncoder backEncoder, DigitalEncoder leftEncoder, DigitalEncoder rightEncoder) {
   double t_now = TimeNow(); // Get a quick time (hopefully not too fast...)
   double t_elapsed = t_now - TimeNow();
   Vleft = (leftEncoder.Counts() / t_elapsed) * r; // Convert encoder counts to average speed
   Vback = (backEncoder.Counts() / t_elapsed) * r;
   Vright = (rightEncoder.Counts() / t_elapsed) * r;
   theta = (double)RPS.Heading(); // Get heading/angle
   xm += Vxm * timeElapsed; // Get new position relative to starting point
   ym += Vym * timeElapsed;
   xw += Vxw * timeElapsed;
   yw += Vyw * timeElapsed;
}

// Move robot w/ pi controller relative to its starting position
void pidMoveAbsolute(double x, double y, double theta, FEHMotor backMotor, FEHMotor leftMotor, FEHMotor rightMotor,
                     DigitalEncoder backEncoder, DigitalEncoder leftEncoder, DigitalEncoder rightEncoder) {
    //bool done = false;
    xTarget = x-RPS.X(); // Hopefully it can handle negative vals
    yTarget = y-RPS.Y();
    thetaTarget = theta;
    while(true) { // hopefully it'll exit
        odometryCalculations(backEncoder, leftEncoder, rightEncoder);
        xError = xTarget - RPS.X();
        yError = yTarget - RPS.Y();
        thetaError = M_PI - normalizeRadian(theta + M_PI - thetaTarget);
        if(isStopPose()) {
            backMotor.Stop();
            leftMotor.Stop();
            rightMotor.Stop();
            return;
        }
        xErrorI += xError;
        yErrorI += yError;
        thetaErrorI += thetaError;
        Vxm = (xError * P) + (xErrorI * I);
        Vym = (yError * P) + (yErrorI * I);
        VxAbs = std::abs(Vxm);
        VyAbs = std::abs(Vym);
        if ((VxAbs > Vmax) || (VyAbs > Vmax)) {
            scale = Vmax / ((VxAbs > VyAbs) ? VxAbs : VyAbs);
            Vxm *= scale;
            Vym *= scale;
        }
        omegap = (thetaError * P) + (thetaErrorI * I);
        if (omegap > omegamax) {
            omegap = omegamax;
        }
        inverseKinematicsMobile();
    }
}

int main(void)
{
    double t_now;
    t_now = TimeNow();
    // Init motors
    FEHMotor backMotor(FEHMotor::Motor0, 9.0);
    FEHMotor leftMotor(FEHMotor::Motor1, 9.0);
    FEHMotor rightMotor(FEHMotor::Motor2, 9.0);

    // Init encoders
    DigitalEncoder backEncoder(FEHIO::P0_0);
    DigitalEncoder leftEncoder(FEHIO::P0_1);
    DigitalEncoder rightEncoder(FEHIO::P0_2);
    backEncoder.ResetCounts();
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Init RPS
    RPS.InitializeTouchMenu();

    pidMoveAbsolute(20, 20, 20, backMotor, leftMotor, rightMotor, backEncoder, leftEncoder, rightEncoder);

    return 0;
}
