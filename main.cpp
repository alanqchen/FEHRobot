#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHBuzzer.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <math.h>
#include <string>

#define MAX_RPM 236.5398195
#define ENCODER_RES 318.0
#define DELTA_T 0.1
#define COUNTS_PER_INCH 35
#define RPS_HEAD_CORR_SPEED 35

float RPS_RAMP_START_X = 0.0;
float RPS_RAMP_START_Y = 0.0;
float RPS_RAMP_START_HEADING = 0.0;

float RPS_LEVERS_X = 0.0;
float RPS_LEVERS_Y = 0.0;
float RPS_LEVERS_HEADING = 0.0;

float RPS_BURGER_X = 0.0;
float RPS_BURGER_Y = 0.0;

float FORWARD_TIME_OUT = 10;

FEHMotor motor1(FEHMotor::Motor0, 9.0);
FEHMotor motor2(FEHMotor::Motor1, 9.0);
FEHMotor motor3(FEHMotor::Motor2, 9.0);

DigitalEncoder motor1_encoder(FEHIO::P0_0);
DigitalEncoder motor2_encoder(FEHIO::P0_2);
DigitalEncoder motor3_encoder(FEHIO::P0_4);

/* NEVER SET ARM SERVO DEGREE ABOVE 150 AND BELOW 10!!!*/
FEHServo jukebox_servo(FEHServo::Servo0);
FEHServo arm_servo(FEHServo::Servo1);

AnalogInputPin CdS_cell(FEHIO::P0_6);

float errorCurr1 = 0.0;
float errorCurr2 = 0.0;
float errorCurr3 = 0.0;

/*!
Inverses the given percent.
 
 @param percent The percent to invert.
 @return The inverted percent.
 */
float InvPercent(float percent) {
    return percent * -1.0;
}

/*!
Stops all motors.

 */
void allStop() {
    motor1.Stop();
    motor2.Stop();
    motor3.Stop();
}

/*!
Limits the given motor percent to be within operating range

 @param percent The percent to limit.
 @return The percent within operating range for the motor.
 */
float limitMotorPercent(float percent) {
    if (percent > 100.0) {
        return 100.0;
    } else if (percent < -100.0) {
        return -100.0;
    } else if(percent < 1 && percent > 0) {
        return 0.0;
    }
    return percent;
}

/*!
Converts and sets radians per second for each motor into their respective percents.
This should only be used in the PI controller.

 @param motor1_RadS Motor 1's speed in radians per second.
 @param motor2_RadS Motor 2's speed in radians per second.
 @param motor3_RadS Motor 3's speed in radians per second.
 */
void setRadSToPercent(float motor1_RadS, float motor2_RadS, float motor3_RadS) {
    float percent1 = (((60.0/(2*M_PI))*motor1_RadS)/MAX_RPM) * 100;
    float percent2 = (((60.0/(2*M_PI))*motor2_RadS)/MAX_RPM) * 100;
    float percent3 = (((60.0/(2*M_PI))*motor3_RadS)/MAX_RPM) * 100;
    percent1 = limitMotorPercent(percent1);
    percent2 = limitMotorPercent(percent2);
    percent3 = limitMotorPercent(percent3);
    if(errorCurr1 < 0.0) {
        motor1.SetPercent(percent1);
    } else {
        motor1.SetPercent(InvPercent(percent1));
    }
    if(errorCurr2 < 0.0) {
        motor2.SetPercent(percent2);
    } else {
        motor2.SetPercent(InvPercent(percent2));
    }
    if(errorCurr3 < 0.0) {
        motor3.SetPercent(percent3);
    } else {
        motor3.SetPercent(InvPercent(percent3));
    }
}

/*!
Converts encoder counts to displacement in radians.

 @param newCount The current interation encoder count.
 @param old The previous iteration encoder count.
 @return The angular displacement in radians.
 */
float countsToRadDisp(int newCount, int old) {
    int difference = newCount-old;
    return (float)difference*(2.0*M_PI)/ENCODER_RES;
}

/*!
Gets the color detected by the CdS cell. The thresholds assume a red filter is used.

 @param start If ``true``, the start light threshold will be used. If ``false``, the jukebox light threshold will be used.
 @return 2 for red, 1 for blue, and 0 for no light.
 */
int getCdsColor(bool start) {
    // Red
    if (start) {
        if(CdS_cell.Value() < 1.5 ) {
            LCD.Clear(RED);
            return 2;
        } else {
            LCD.Clear(FEHLCD::Green);
            return 0;
        }
    } else {
        if(CdS_cell.Value() < 1.1 ) {
            LCD.Clear(RED);
            return 2;
        // Blue
        } else {
            LCD.Clear(BLUE);
            return 1;
        }
    }
}

/*! 
Uses a PI controller to move the robot using a trajectory profile as the reference data.
The passed in trajectory profile should have 6 columns. The first 3 are the total angular
displacements(rad) of motors 1, 2, and 3 repsectively, with the last 3 being the respective angular
velocities(rad/s). The delta time is 0.1 seconds. The loop each iteration will calculate difference in
encoder counts to convert it into displacement in radians using countsToRadDisp. This will then be
added to the total angular displacement for each motor, and these calculated values will be subracted
with the reference angular displacements to get the error for each motor. These errors are then summed
to the motors individual total error counter, and based on the error values and P & I constants,
angular velocities(rad/s) to set the motors to is calculated. Then the funtion setRadSToPercent is called to
convert the angular speeds to percent and limit them within operating range, and then set the motors to that percent.

 @param fName the file name of trajectory profile.
 @param size the number of lines/commands in trajectory profile.
 @param preload If true, the file will be preloaded and won't start until the start light turns on.
 */
void PIMoveTo(char* fName, int size, bool preload) {
    /* Set important variables */
    int countNew1 = 0;
    int countNew2 = 0;
    int countNew3 = 0;
    int countOld1 = 0;
    int countOld2 = 0;
    int countOld3 = 0;
    float displacement1 = 0.0;
    float displacement2 = 0.0;
    float displacement3 = 0.0;
    float refSpeed1;
    float refSpeed2;
    float refSpeed3;
    float phi1 = 0.0;
    float phi2 = 0.0;
    float phi3 = 0.0;
    float motorSpeed1 = 0.0; 
    float motorSpeed2 = 0.0; 
    float motorSpeed3 = 0.0; 
    float errorTotal1 = 0.0;
    float errorTotal2 = 0.0;
    float errorTotal3 = 0.0;
    float Kp = 20.0;
    float Ki = 1.0;

    /* Get trajectory profile from file */
    FEHFile *fptr = SD.FOpen(fName,"r");
    /* Open write files to track error and delta angular displacement */
    // This is useful for tuning among other things
    FEHFile *fOutErrptr = SD.FOpen("errorLog.txt","w");
    FEHFile *fOutDispptr = SD.FOpen("dispLog.txt","w");
    FEHFile *fOutVelptr = SD.FOpen("velLog.txt","w");
    
    /* Init 2d arrays to store reference data and other temp variables to read from file */
    float pos_ref[3][size];
    float vel_ref[3][size];
    float refPos1;
    float refPos2;
    float refPos3;
    /* If file failed to open, or invalid profile, return and make the screen red */
    if(SD.FEof(fptr)) {
        LCD.Clear(FEHLCD::Red);
        return;
    }
    /* Parse trajectory file */
    int i = 0;
    while(!SD.FEof(fptr)) {
        SD.FScanf(fptr, "%f%f%f%f%f%f", &refPos1, &refPos2, &refPos3, &refSpeed1, &refSpeed2, &refSpeed3);
        pos_ref[0][i] = refPos1;
        pos_ref[1][i] = refPos2;
        pos_ref[2][i] = refPos3;
        vel_ref[0][i] = refSpeed1;
        vel_ref[1][i] = refSpeed2;
        vel_ref[2][i] = refSpeed3;
        i++;
    }
    if(size < i) {
        LCD.Clear(FEHLCD::Red);
        return;
    }
    size = i;
    /* Close trajectory file */
    SD.FClose(fptr);
    /* PRELOAD LOOP */
    if(preload) {
        // Set green to show it's ready
        LCD.Clear(FEHLCD::Green);
        while(getCdsColor(true) == 0); // wait until a light turns on
    }
    /* Reset encoder counts */
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    /* PI LOOP */
    // Yes, not PID as the derivative term isn't needed currently
    for (int i = 0; i < size; i++) {
        /* Get new encoder counts */
        countNew1 = motor1_encoder.Counts();
        countNew2 = motor2_encoder.Counts();
        countNew3 = motor3_encoder.Counts();

        if(errorCurr1 < 0.0) {
            displacement1 = countsToRadDisp(countNew1, countOld1) * -1;
        } else {
            displacement1 = countsToRadDisp(countNew1, countOld1);
        }
        if(errorCurr2 < 0.0) {
            displacement2 = countsToRadDisp(countNew2, countOld2) * -1;
        } else {
            displacement2 = countsToRadDisp(countNew2, countOld2);
        }
        if(errorCurr3 < 0.0) {
            displacement3 = countsToRadDisp(countNew3, countOld3) * -1;
        } else {
            displacement3 = countsToRadDisp(countNew3, countOld3);
        }
        // Set old counts to new counts for the next iteration
        countOld1 = countNew1;
        countOld2 = countNew2;
        countOld3 = countNew3;
        // Add to total angular displacement
        phi1 += displacement1;
        phi2 += displacement2;
        phi3 += displacement3;
        
        // Write to log file
        SD.FPrintf(fOutDispptr, "%f\t%f\t%f\n", displacement1, displacement2, displacement3);
        
        /* Calculate current error relative to reference angular positions for each encoder */
        errorCurr1 = pos_ref[0][i] - phi1;
        errorCurr2 = pos_ref[1][i] - phi2;
        errorCurr3 = pos_ref[2][i] - phi3;
        
        // Saftey check in case something goes terribly wrong
        if(errorCurr1 > 3)
            return;

        // Write errors to log file
        SD.FPrintf(fOutErrptr, "%f\t%f\t%f\n", errorCurr1, errorCurr2, errorCurr3);
        // Add to total error (for integral term)
        errorTotal1 += errorCurr1;
        errorTotal2 += errorCurr2;
        errorTotal3 += errorCurr3;
        
        /* Calc motor speeds (rad/s) using P and I */
        motorSpeed1 = Kp * errorCurr1 + Ki * DELTA_T * (errorTotal1);
        motorSpeed2 = Kp * errorCurr2 + Ki * DELTA_T * (errorTotal2);
        motorSpeed3 = Kp * errorCurr3 + Ki * DELTA_T * (errorTotal3);

        /* Use the reference velocities to determine if motor speed should change signs */
        if(vel_ref[0][i] < 0.0 || (errorCurr1 < 0 && motorSpeed1 < 0)) {
            motorSpeed1 *= -1.0;
        }
        if(vel_ref[1][i] < 0.0 || (errorCurr2 < 0 && motorSpeed2 < 0)) {
            motorSpeed2 *= -1.0;
        }
        if(vel_ref[2][i] < 0.0 || (errorCurr3 < 0 && motorSpeed3 < 0)) {
            motorSpeed3 *= -1.0;
        }

        SD.FPrintf(fOutVelptr, "%f\t%f\t%f\n", motorSpeed1, motorSpeed2, motorSpeed3);
        /* Set motors to speed */
        setRadSToPercent(motorSpeed1, motorSpeed2, motorSpeed3);
        /* Wait 0.1 seconds (100 miliseconds) */
        Sleep(100);
    }
    /* Done with trajectory profile, stop all motors */
    allStop();
    /* Close all log files */
    SD.FClose(fOutErrptr);
    SD.FClose(fOutDispptr);
    SD.FClose(fOutVelptr);
}

/*!
Rotates counter-clockwise, when given a positve percent.

 @param percent the power for motors. A negative value will move clockwise.
 @param degree the amount of degrees to rotate. Must be positive.
 */
void rotateCC(float percent, int degree) {
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor1.SetPercent(InvPercent(percent));
    motor2.SetPercent(InvPercent(percent));
    motor3.SetPercent(InvPercent(percent));

    while((motor1_encoder.Counts() + motor2_encoder.Counts()+ motor3_encoder.Counts())/3< 2.69420*degree);
    allStop();

}

/*!
Moves forward in the direction of motors 2 & 3 for the given inch distance.

 @param percent the power for motors 2 & 3. A negative value will move backwards.
 @param inch the distance to move. Must be positive.
 */
void forward23(float percent, int inch) {
    float start = TimeNow();
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor2.SetPercent(InvPercent(percent));
    motor3.SetPercent(percent);

    while(TimeNow() - start < FORWARD_TIME_OUT && (motor2_encoder.Counts() + motor3_encoder.Counts())/2< COUNTS_PER_INCH*inch);
    allStop();
}

/*!
Moves forward in the direction of motors 3 & 1 for the given inch distance.

 @param percent the power for motors 3 & 1. A negative value will move backwards.
 @param inch the distance to move. Must be positive.
 */
void forward31(float percent, float inch) {
    float start = TimeNow();
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor3.SetPercent(InvPercent(percent));
    motor1.SetPercent(percent);

    while(TimeNow() - start < FORWARD_TIME_OUT && (motor1_encoder.Counts() + motor3_encoder.Counts())/2< COUNTS_PER_INCH*inch);
    allStop();
}

/*!
Moves forward in the direction of motors 1 & 2 for the given inch distance.

@param percent the power for motors 1 & 2. A negative value will move backwards.
@param inch the distance to move. Must be positive.
 */
void forward12(float percent, float inch) {
    float start = TimeNow();
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor1.SetPercent(InvPercent(percent));
    motor2.SetPercent(percent);

    while(TimeNow() - start < FORWARD_TIME_OUT && (motor1_encoder.Counts() + motor2_encoder.Counts())/2< COUNTS_PER_INCH*inch);
    allStop();
}

/*!
Moves forward in the x axis direction, relative to the robot's local orientation.

 @param percent the power for motor 3. Motors 1 & 2 will be set to percent/2. A negative value will move backwards.
 @param inch the distance to move. Must be positive.
 */
void forwardX(float percent, float inch) {
    float start = TimeNow();
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor1.SetPercent(percent/2.0);
    motor2.SetPercent(percent/2.0);
    motor3.SetPercent(InvPercent(percent));

    while(TimeNow() - start < FORWARD_TIME_OUT && motor3_encoder.Counts() < COUNTS_PER_INCH*inch);
    allStop();
}

/*!
Uses RPS to correct the heading, relative to the global (course) coordinates.

 @param finalHeading the desired heading. 0 <= finalHeading < 359.9
 @param power the motor percent speeds. A low power will be accurate, but slow. A high power will be fast, but not as accurate.
 */
void correctHeading(float finalHeading, float power) {
    float currHeading;
    currHeading = RPS.Heading();
    while (currHeading < 0) {
        currHeading = RPS.Heading();
        LCD.Clear(FEHLCD::Red);
        Sleep(50);
    }
    float angle = fabsf(finalHeading - currHeading);

    if(angle < 180) {
        if (currHeading > finalHeading) {
                power *= -1.0;
        }
    } else {
        angle = 360 - angle;
        if (currHeading < finalHeading) {
            power *= -1.0;
        }
    }
    rotateCC(power, angle);
    Sleep(100);
}

/*!
This is the function that runs the final "acutal" course. Note that because the
testing time was cut short, only the jukebox, tray, lever, and burger tasks could
be implemented.

 */
void actual() {
    //go from start to jukebox light, press correct button, go to from of ramp
    PIMoveTo("toJL.txt", 21, true);
    Sleep(100);
    int cdsValue = getCdsColor(false);
    correctHeading(270, RPS_HEAD_CORR_SPEED);
    if(cdsValue == 2) {
        PIMoveTo("toRed.txt", 61, false);
        PIMoveTo("rToRamp.txt", 61, false);
    } else if(cdsValue == 1) {
        PIMoveTo("toBlue.txt", 31, false);
        PIMoveTo("bToRamp.txt", 31, false);
    }

    //correct heading and X before ramp
    correctHeading(RPS_RAMP_START_HEADING, 1.33*RPS_HEAD_CORR_SPEED);
    Sleep(500);
    float currX = RPS.X();
    // OLD: 18.3
    float deltaX = fabsf(RPS_RAMP_START_X - currX);
    if (RPS_RAMP_START_X > currX) {
        forward12(RPS_POS_CORR_SPEED, deltaX);
    } else {
        forward12(-RPS_POS_CORR_SPEED, deltaX);
    }

    //go up ramp
    PIMoveTo("upRamp3.txt", 31, false);

    //align X, Y, and heading at sink
    Sleep(100);
    correctHeading(RPS_RAMP_START_HEADING, RPS_HEAD_CORR_SPEED);
    Sleep(500);
    float currY = RPS.Y();
    float deltaY = fabsf(41.4 - currY);
    if (41.4 > currY) {
        forwardX(-RPS_POS_CORR_SPEED, deltaY);
    } else {
        forwardX(RPS_POS_CORR_SPEED, deltaY);
    }
    correctHeading(0, RPS_HEAD_CORR_SPEED);
    Sleep(500);
    currX = RPS.X();
    deltaX = fabsf(15.5 - currX);
    FORWARD_TIME_OUT = 1.75;
    if (15.5 > currX) {
        forward12(RPS_POS_CORR_SPEED, deltaX);
    } else {
        forward12(-RPS_POS_CORR_SPEED, deltaX);
    }
    FORWARD_TIME_OUT = 10;

    //throw tray
    arm_servo.SetDegree(60);
    Sleep(150);
    arm_servo.SetDegree(80);
    Sleep(50);
    arm_servo.SetDegree(100);
    Sleep(50);
    arm_servo.SetDegree(120);
    Sleep(10);
    arm_servo.SetDegree(115);
    Sleep(10);
    arm_servo.SetDegree(110);
    Sleep(50);
    correctHeading(0, RPS_HEAD_CORR_SPEED);
    arm_servo.SetDegree(80);
    Sleep(100);

    //center between three levers from side of sink
    PIMoveTo("toLevers.txt", 31, false);
    arm_servo.SetDegree(110);
    Sleep(500);
    currX = RPS.X();
    // OLD 20.4
    deltaX = fabsf(RPS_LEVERS_X - currX);
    if (RPS_LEVERS_X > currX) {
        forward12(RPS_POS_CORR_SPEED, deltaX);
    } else {
        forward12(-RPS_POS_CORR_SPEED, deltaX);
    }
    // OLD 345
    correctHeading(RPS_LEVERS_HEADING, RPS_HEAD_CORR_SPEED);
    Sleep(500);
    currY = RPS.Y();
    // OLD: 57.2
    deltaY = fabsf(RPS_LEVERS_Y - currY);
    if (RPS_LEVERS_Y > currY) {
        forward23(RPS_POS_CORR_SPEED, deltaY);
    } else {
        forward23(-RPS_POS_CORR_SPEED, deltaY);
    }


    //flip correct lever down and move near burger
    int lever = RPS.GetIceCream();
    if (lever == 0) {
        PIMoveTo("toL1.txt", 31, false);
        leverDown();
        PIMoveTo("L1toBgr.txt", 31, false);
    } else if (lever == 1) {
        PIMoveTo("toL2.txt", 31, false);
        leverDown();
        PIMoveTo("L2toBgr.txt", 31, false);
    } else {
        PIMoveTo("toL2.txt", 31, false);
        forward31(-35, 3.75);
        leverDown();
        Sleep(200);
        forward31(35, 3.75);
        PIMoveTo("L2toBgr.txt", 31, false);
    }

    //move to and align to burger
    arm_servo.SetDegree(8);
    correctHeading(180, RPS_HEAD_CORR_SPEED);
    Sleep(500);
    currX = RPS.X();
    // OLD: 29.7
    deltaX = fabsf(RPS_BURGER_X - currX);
    if (RPS_BURGER_X > currX) {
        forward12(-RPS_POS_CORR_SPEED, deltaX);
    } else {
        forward12(RPS_POS_CORR_SPEED, deltaX);
    }
    correctHeading(200, RPS_HEAD_CORR_SPEED);
    FORWARD_TIME_OUT = 1.75;
    Sleep(500);
    currY = RPS.Y();
    forward31(RPS_POS_CORR_SPEED, RPS_BURGER_Y - currY);
    FORWARD_TIME_OUT = 10;

    //flip burger
    arm_servo.SetDegree(85);
    Sleep(2000);
    arm_servo.SetDegree(8);
}

/*!
Runs the sequence to calibrate the RPS coordinates for the ramp, levers, and burger task.
This alters the global variables to store the coordinates.
To signal when it is ready to set a coordinate, the screen will change color:
    * Black - Ramp,
    * Gray - Levers,
    * Scarlet - Burger,
    * Blue - Tap when on starting position.

Additonally, a high pitch will sound when it detects a tap, followed by a low pitch
sound when it's ready for the next tap.
 */
void setupRPS() {
    Sleep(1000);
    float trash_x;
    float trash_y;
    LCD.Clear(FEHLCD::Black);
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("TAP FOR RAMP POS");
    while(!LCD.Touch(&trash_x, &trash_y));
    Buzzer.Tone( FEHBuzzer::G5,  300 );
    RPS_RAMP_START_X = RPS.X();
    while(RPS_RAMP_START_X == -1.0) {
        RPS_RAMP_START_X = RPS.X();
    }
    RPS_RAMP_START_Y = RPS.Y();
    RPS_RAMP_START_HEADING = RPS.Heading();
    Buzzer.Tone( FEHBuzzer::D6,  300 );
    Sleep(500);

    LCD.Clear(FEHLCD::Gray);
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("TAP FOR LEVERS POS");
    while(!LCD.Touch(&trash_x, &trash_y));
    Buzzer.Tone( FEHBuzzer::G5,  300 );
    RPS_LEVERS_X = RPS.X();
    while(RPS_LEVERS_X == -1.0) {
        RPS_LEVERS_X = RPS.X();
    }
    RPS_LEVERS_Y = RPS.Y();
    RPS_LEVERS_HEADING = RPS.Heading();
    Buzzer.Tone( FEHBuzzer::D6,  300 );
    Sleep(500);

    LCD.Clear(FEHLCD::Scarlet);
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("TAP FOR BURGER POS");
    while(!LCD.Touch(&trash_x, &trash_y));
    Buzzer.Tone( FEHBuzzer::G5,  300 );
    RPS_BURGER_X = RPS.X();
    while(RPS_BURGER_X == -1.0) {
        RPS_BURGER_X = RPS.X();
    }
    RPS_BURGER_Y = RPS.Y();
    Buzzer.Tone( FEHBuzzer::D6,  300 );
    Sleep(500);

    LCD.Clear(FEHLCD::Blue);
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("-");
    LCD.WriteLine("TAP TO LOAD");
    while(!LCD.Touch(&trash_x, &trash_y));
    Buzzer.Tone( FEHBuzzer::G5,  300 );
}

/*!
This is the "controller" for the program. It runs all the setup functions and
calls the function that runs the course.
 */
int main(void)
{
    jukebox_servo.SetMin(700);
    jukebox_servo.SetMax(2380);
    arm_servo.SetMin(508);
    arm_servo.SetMax(2464);
    arm_servo.SetDegree(8);
    jukebox_servo.SetDegree(5.0);
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    RPS.InitializeTouchMenu();
    setupRPS();
    arm_servo.SetDegree(45);
    actual();

    while (true) {
        LCD.Write(RPS.X());
        LCD.Write("  ");
        LCD.Write(RPS.Y());
        LCD.Write(" ");
        LCD.WriteLine(RPS.Heading());
    }

    return 0;
}
