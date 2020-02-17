#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <math.h>
#include "simplemovement.h"

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
