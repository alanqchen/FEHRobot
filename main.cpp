#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <math.h>
#include <string>

#define MOTOR_ANGLE_1 0.0*M_PI/180.0
#define MOTOR_ANGLE_2 120*M_PI/180.0
#define MOTOR_ANGLE_3 240*M_PI/180.0
#define MAX_RPM 236.5398195
#define ENCODER_RES 318.0
#define DELTA_T 0.1
#define R 0.11176
#define r 0.03429
#define COUNTS_PER_INCH 35
#define ERROR_MARGIN 3
#define DIST_ERROR_MARGIN 1
#define NUM_CORR_ITERATIONS 2

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

float InvPercent(float percent) {
    return percent * -1.0;
}

// convert degrees from RPS to radian
float degreeToRadian(float degree) {
    return degree * M_PI/180.0;
}

// convert degrees from RPS to radian
float radianToDegree(float radian) {
    return radian * 180.0/M_PI;
}

// Stops all motors
void allStop() {
    motor1.Stop();
    motor2.Stop();
    motor3.Stop();
}

// Limits motor percent to be within operating range
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

// Converts radians per second to percent
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

/*
 * xVel - m/s
 * theta - rad
 * phi - rad/s
 */
// Movement using kinematic equations
void kinematics(float xVel, float yVel, float theta, float phi) {
    float angVel1 = (-1 * sin(theta+MOTOR_ANGLE_1)*cos(theta)*xVel+cos(theta+MOTOR_ANGLE_1)*cos(theta)*yVel+R*phi)/r;
    float angVel2 = (-1 * sin(theta+MOTOR_ANGLE_2)*cos(theta)*xVel+cos(theta+MOTOR_ANGLE_2)*cos(theta)*yVel+R*phi)/r;
    float angVel3 = (-1 * sin(theta+MOTOR_ANGLE_3)*cos(theta)*xVel+cos(theta+MOTOR_ANGLE_3)*cos(theta)*yVel+R*phi)/r;
    setRadSToPercent(angVel1, angVel2, angVel3);
}

// vel - 0.215-> 25% 0.85 -> 100%
// generally keep UNDER .7; .5 or under to be safe
void moveForwardBackward(float vel, float theta) {
    kinematics(0.0, vel, theta+M_PI/6.0, 0.0);
}

// .7 MAX
void moveLeftRight(float vel, float theta) {
    kinematics(vel, 0.0, theta+M_PI/6.0, 0.0);
}

void spinClockwise(float vel, float theta) {
    kinematics(0.0, vel, theta+M_PI/6.0, 0.0);
}

// postive for counter clockwise
// 6.6 MAX, 1.65-> 25%
void spin(float phi, float theta) {
    kinematics(0.0, 0.0, theta+M_PI/6.0, phi);
}

float inchestoMeters(float num) {
    return num/39.37;
}

float countsToRadDisp(int newCount, int old) {
    int difference = newCount-old;
    //float degDif = (float)difference * 360.0/318.0;
    //return degDif * (M_PI / 180.0);
    return (float)difference*(2.0*M_PI)/ENCODER_RES;
}

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

// TODO: Change name to PIMoveTO
/*! 
\rst

This is some funky non-xml compliant text: <& !><

.. note::

   This reStructuredText has been handled correctly.
\endrst
\verbatim
void test() {
    // This is test code
}
\endverbatim
Uses a PI controller to move the robot using a trajectory profile as the reference data.
The passed in trajectory profile should have 6 columns. The first 3 are the total angular
displacements(rad) of motors 1, 2, and 3 repsectively, with the last 3 being the respective angular
velocities(rad/s). Right now, delta time is 0.1 seconds. The loop each iteration will calculate difference in
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
void PIDMoveTo(char* fName, int size, bool preload) {
    //float xMeters = inchestoMeters(x);
    //float yMeters = inchestoMeters(y);
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
    float phiVel1 = 0.0;
    float phiVel2 = 0.0;
    float phiVel3 = 0.0;
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
    float Ki = 2.0;
    float Kd = 0.0;
    float pidMarginError = 0.1; // in inches
    // might remove this
    bool setup = true;
    


    //int len = fName.length();
    //char charArr[len+1];
    //strcpy(charArr, fName.c_str());
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
    float temp1;
    float temp2;
    float temp3;
    /* If file failed to open, or invalid profile, return and make the screen red */
    if(SD.FEof(fptr)) {
        LCD.Clear(FEHLCD::Red);
        return;
    }
    /* Parse trajectory file */
    int i = 0;
    while(!SD.FEof(fptr)) {
        SD.FScanf(fptr, "%f%f%f%f%f%f", &temp1, &temp2, &temp3, &refSpeed1, &refSpeed2, &refSpeed3);
        pos_ref[0][i] = temp1;
        pos_ref[1][i] = temp2;
        pos_ref[2][i] = temp3;
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
        /* Check if first iteration (no old values) */
        // This is redundant! (remove later)
        /*
        if(setup) {
            displacement1 = countsToRadDisp(countNew1, 0);
            displacement2 = countsToRadDisp(countNew2, 0);
            displacement3 = countsToRadDisp(countNew3, 0);
            setup = false;
        } else {
            displacement1 = countsToRadDisp(countNew1, countOld1);
            displacement2 = countsToRadDisp(countNew2, countOld2);
            displacement3 = countsToRadDisp(countNew3, countOld3);
        }
        */
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
        // Calc average angular velocity (this will be used if we switch to a angular velocity trajectory profile)
        phiVel1 = displacement1/DELTA_T;
        phiVel2 = displacement2/DELTA_T;
        phiVel3 = displacement3/DELTA_T;
        
        // Write to log file
        SD.FPrintf(fOutDispptr, "%f\t%f\t%f\n", displacement1, displacement2, displacement3);

        // TODO: IMPLEMENT DEAD BAND
        
        /* Calculate current error relative to reference angular positions for each encoder */
        errorCurr1 = pos_ref[0][i] - phi1;
        errorCurr2 = pos_ref[1][i] - phi2;
        errorCurr3 = pos_ref[2][i] - phi3;
        
        // Saftey check in case something goes terribly wrong, may or may not be needed later
        if(errorCurr1 > 3)
            return;


        // Write errors to log file
        SD.FPrintf(fOutErrptr, "%f\t%f\t%f\n", errorCurr1, errorCurr2, errorCurr3);
        // Add to total error (for integral term)
        errorTotal1 += errorCurr1;
        errorTotal2 += errorCurr2;
        errorTotal3 += errorCurr3;
        
        // TODO: ADD INTERGRAL WINDUP REMOVAL
        
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

        //motor1.SetPercent(limitMotorPercent(motorSpeed1));
        //motor2.SetPercent(limitMotorPercent(motorSpeed2));
        //motor3.SetPercent(limitMotorPercent(motorSpeed3));
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

void moveForward(float percent, float inch) {
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor1.SetPercent(InvPercent(percent));
    motor2.SetPercent(percent);

    while((motor1_encoder.Counts() + motor2_encoder.Counts())/2< 35*inch);
    allStop();

}

void performance1() {
    moveForward(25, 18);
    allStop();
    // Read Cds Cell
    int cdsValue = getCdsColor(false);
    Sleep(1.5);
    // If blue
    if(cdsValue == 1)  {
        rotateCC(25, 106);
        Sleep(1.5);
        moveForward(25, 4);
    // If red
    } else if (cdsValue == 2) {
        moveForward(25, 3);
        Sleep(1.5);
        rotateCC(25, 107);
        Sleep(1.5);
        moveForward(25, 5.25);
    }
    Sleep(1.5);
    // Move backward from jukebox
    moveForward(-25, 6);
    jukebox_servo.SetDegree(5.0);
    Sleep(0.5);
    // Rotate to align heading to ramp starting position
    rotateCC(25, 90);

    Sleep(0.5);
    // Move forward to ramp starting position
    if(cdsValue == 1) {
        moveForward(25, 6);
    } else if(cdsValue == 2) {
        moveForward(25, 10);
    }
    Sleep(0.5);
    // Rotate towards ramp
    rotateCC(25, 90);
    Sleep(0.5);
    // Go up ramp
    moveForward(75, 30);
    Sleep(0.5);
    moveForward(-25, 30);
    Sleep(0.5);
}

void sinkDump() {
    for(int i = 45; i <= 140; i+=5) {
        arm_servo.SetDegree(i);
        Sleep(100);
    }
    for(int i = 140; i >= 45; i-=5) {
        arm_servo.SetDegree(i);
        Sleep(100);
    }
}

/*!
Moves forward in the direction of motors 2 & 3 for the given inch distance

@param percent the power for motors 2 & 3.
@param inch the distance to move.
 */
void forward23(float percent, int inch) {
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor2.SetPercent(InvPercent(percent));
    motor3.SetPercent(percent);

    while((motor2_encoder.Counts() + motor3_encoder.Counts())/2< COUNTS_PER_INCH*inch);
    allStop();
}

/*!
Moves forward in the direction of motors 3 & 1 for the given inch distance

@param percent the power for motors 3 & 1.
@param inch the distance to move.
 */
void forward31(float percent, float inch) {
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor3.SetPercent(InvPercent(percent));
    motor1.SetPercent(percent);

    while((motor1_encoder.Counts() + motor3_encoder.Counts())/2< COUNTS_PER_INCH*inch);
    allStop();
}

/*!
Moves forward in the direction of motors 1 & 2 for the given inch distance

@param percent the power for motors 1 & 2.
@param inch the distance to move.
 */
void forward12(float percent, float inch) {
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    motor1.SetPercent(InvPercent(percent));
    motor2.SetPercent(percent);

    while((motor1_encoder.Counts() + motor2_encoder.Counts())/2< COUNTS_PER_INCH*inch);
    allStop();
}

void correctHeading(float finalHeading, float power) {
    float currHeading;
    for(int i = 0; i < 1; i++) {
        currHeading = RPS.Heading();
        while (currHeading < 0) {
            currHeading = RPS.Heading();
            Sleep(50);
        }
        float angle = fabsf(finalHeading - currHeading);

        if (currHeading == -1) {
            LCD.Clear(FEHLCD::Red);
            return;
        }

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
}

void RPSCorrectError(float finalX, float finalY, float finalHeading) {
    float currHeading = RPS.Heading();
    float currX = RPS.X();
    float currY = RPS.Y();


    float deltaX = finalX - currX;
    if(deltaX == 0.0) {
        deltaX += 0.0000000000000000000001;
    }
    float deltaY = finalY - currY;
    if(deltaY == 0.0) {
        deltaY += 0.0000000000000000000001;
    }
    float theta;
    // Q1
    if(deltaX >= 0 && deltaY >= 0) {
        theta = radianToDegree(atan((deltaY)/(deltaX)));
    // Q2
    } else if(deltaX < 0 && deltaY >=0) {
        theta = radianToDegree(atan((-1.0*deltaX)/(deltaY)));
        theta += 90.0;
    // Q3
    } else if(deltaX < 0 && deltaY < 0) {
        theta = radianToDegree(atan((-1.0*deltaY)/(-1.0*deltaX)));
        theta += 180;
    // Q4
    } else {
        theta = radianToDegree(atan((deltaX)/(-1.0*deltaY)));
        theta += 270;
    }

    // THIS IS PROB WRONG
    float deltaTheta = fabsf(currHeading - theta);
    if (deltaTheta > 180) {
        deltaTheta = 360 - deltaTheta;
    }
    float power = 20.0;
    if(deltaTheta < 90.0) {
        correctHeading(deltaTheta, 18.0);
    } else {
        deltaTheta += 180.0;
        if(deltaTheta > 360.0) {
            deltaTheta -= 360;
        }
        correctHeading(deltaTheta, 18.0);
        power = -20.0;
    }

    float dist = sqrtf(deltaX * deltaX + deltaY * deltaY);
    forward12(power, dist);

    correctHeading(finalHeading, 18.0);
}

void performance2() {
    arm_servo.SetDegree(65);
    PIDMoveTo("start1.txt", 31, true);
    for(int i = 65; i >= 45; i-=2) {
        arm_servo.SetDegree(i);
        Sleep(10);
    }
    //rotateCC(-25, 120);
    //rotateCC(-25, 90);
    //forward23(75, 34);
    PIDMoveTo("mR34.txt", 31, false);



    Sleep(0.5);
    //PIDMoveTo("r90CW.txt", 31);
    rotateCC(-25, 90);
    Sleep(0.5);
    PIDMoveTo("toSink.txt", 31, false);


    Sleep(0.5);

    //rotateCC(25, 120);
    //PIDMoveTo("toSink.txt", 31);
    sinkDump();
    //PIDMoveTo("toSlide.txt", 31);
    rotateCC(-25, 152);

    Sleep(0.5);

    forward31(25, 24.0);
    forward31(-25, .1);

    //PIDMoveTo("rotate6.txt", 11, false);
    rotateCC(-25, 30);

    arm_servo.SetDegree(90.0);
    jukebox_servo.SetDegree(170.0);
    Sleep(0.75);
    forward12(25, 5);
    jukebox_servo.SetDegree(160.0);
    Sleep(0.5);
    PIDMoveTo("slideT.txt", 26, false);
    Sleep(0.5);
    forward12(-25, 23);
}

void performance3() {
    // to center of ramp
    PIDMoveTo("start1.txt", 31, true);

    //up ramp
    PIDMoveTo("mR34.txt", 31, false);

    //east to wall
    forward12(-25, 17);

    //west off wall
    //forward12(25, 1.5);
    forward12(25, .75);

    //rotate towards burger
    rotateCC(25, 22);
    //PIDMoveTo("ss.txt", 6, false);

    arm_servo.SetDegree(0);

    //north towards burger
    forward31(25, 8.1);
    // back off from burger
    forward31(-25, .15);
    arm_servo.SetDegree(55);
    Sleep(1.5);
    arm_servo.SetDegree(0);
    Sleep(1.5);

    //south from burger
    forward31(-25, 4);

    //face ice cream
    rotateCC(-25, 23);

    arm_servo.SetDegree(70);

    //move diagonally to lever
    PIDMoveTo("toLever.txt", 31, false);

    arm_servo.SetDegree(40);
    Sleep(1.5);
    arm_servo.SetDegree(70);
}

void leverDown() {
    arm_servo.SetDegree(40);
    Sleep(0.300);
    arm_servo.SetDegree(70);
}

void actual() {
    arm_servo.SetDegree(65);
    PIDMoveTo("toJL.txt", 21, true);
    Sleep(0.100);
    int cdsValue = getCdsColor(false);
    if(cdsValue == 2) {
        PIDMoveTo("toRed.txt", 61, false);
        PIDMoveTo("rToRamp.txt", 61, false);
    } else if(cdsValue == 1) {
        PIDMoveTo("toBlue.txt", 31, false);
        PIDMoveTo("bToRamp.txt", 31, false);
    }

    correctHeading(180, 30);

    float deltaX = 19 - RPS.X();
    forward12(18, -1.0*deltaX);

    PIDMoveTo("upRamp.txt", 31, false);

    deltaX = 18 - RPS.X();
    forward12(18, -1.0*deltaX);

    correctHeading(90, 25);

    PIDMoveTo("toSink.txt", 31, false);
    sinkDump();

    correctHeading(170, 30);
    arm_servo.SetDegree(70);
    int lever = RPS.GetIceCream();

    /*
    if (lever == 1) {
        PIDMoveTo("toL1.txt", 31, false);
    } else if (lever == 2) {
        PIDMoveTo("toL2.txt", 31, false);
    } else {
        PIDMoveTo("toL3.txt", 31, false);
    }
    */
    PIDMoveTo("toL2.txt", 31, false);

    leverDown();

}

int main(void)
{
    jukebox_servo.SetMin(700);
    jukebox_servo.SetMax(2380);
    arm_servo.SetMin(508);
    arm_servo.SetMax(2464);
    arm_servo.SetDegree(65);
    jukebox_servo.SetDegree(5.0);
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();
    RPS.InitializeTouchMenu();
    actual();

    return 0;
}
