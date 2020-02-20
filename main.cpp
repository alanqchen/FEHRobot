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


FEHMotor motor1(FEHMotor::Motor0, 9.0);
FEHMotor motor2(FEHMotor::Motor1, 9.0);
FEHMotor motor3(FEHMotor::Motor2, 9.0);

DigitalEncoder motor1_encoder(FEHIO::P0_0);
DigitalEncoder motor2_encoder(FEHIO::P0_2);
DigitalEncoder motor3_encoder(FEHIO::P0_4);

FEHServo jukebox_servo(FEHServo::Servo0);
FEHServo arm_servo(FEHServo::Servo1);

AnalogInputPin CdS_cell(FEHIO::P0_6);

float InvPercent(float percent) {
    return percent * -1.0;
}
/*
float* parseTrajectoryFile(std::string fileName, int size) {
    FEHFile *fptr = SD.FOpen(fileName,"w");
    float pos_res[3][size];
    float temp1;
    float temp2;
    float temp3;
    if(!SD.FEof(fptr)) {
        LCD.Clear(FEHLCD::Red);
        return NULL;
    }

    for(int i=0; i<size; i++) {
        SD.FScanf(fptr, "%f%f%f", &temp1, &temp2, &temp3);
        pos_res[0][i] = temp1;
        pos_res[1][i] = temp2;
        pos_res[2][i] = temp3;
    }
    SD.FClose(fptr);
    return pos_res;
}
*/

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
    motor1.SetPercent(InvPercent(percent1));
    motor2.SetPercent(InvPercent(percent2));
    motor3.SetPercent(InvPercent(percent3));
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

float countsToRadDisp(int newCount, int old) {
    int difference = newCount-old;
    //float degDif = (float)difference * 360.0/318.0;
    //return degDif * (M_PI / 180.0);
    return (float)difference*(2.0*M_PI)/ENCODER_RES;
}

// TODO: Change name to PIMoveTO
/* PIDMoveTO
 * Uses a PI controller to move the robot using a trajectory profile as the reference data.
 *  The passed in trajectory profile should have 6 columns. The first 3 are the total angular
 *  displacements(rad) of motors 1, 2, and 3 repsectively, with the last 3 being the respective angular
 *  velocities(rad/s). Right now, delta time is 0.1 seconds. The loop each iteration will calculate difference in
 *  encoder counts to convert it into displacement in radians using countsToRadDisp. This will then be
 *  added to the total angular displacement for each motor, and these calculated values will be subracted
 *  with the reference angular displacements to get the error for each motor. These errors are then summed
 *  to the motors individual total error counter, and based on the error values and P & I constants,
 *  angular velocities(rad/s) to set the motors to is calculated. Then the funtion setRadSToPercent is called to
 *  convert the angular speeds to percent and limit them within operating range, and then set the motors to that percent.
 *
 * Parameters:
 *  fName - file name of trajectory profile  
 *  size - number of lines/commands in trajectory profile
 */
void PIDMoveTo(char* fName, int size) {
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
    float errorCurr1 = 0.0;
    float errorCurr2 = 0.0;
    float errorCurr3 = 0.0;
    float errorTotal1 = 0.0;
    float errorTotal2 = 0.0;
    float errorTotal3 = 0.0;
    float Kp = 20.0;
    float Ki = 0.0;
    float Kd = 0.0;
    float pidMarginError = 0.1; // in inches
    // might remove this
    bool setup = true;
    
    /* Reset encoder counts */
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();


    //int len = fName.length();
    //char charArr[len+1];
    //strcpy(charArr, fName.c_str());
    /* Get trajectory profile from file */
    FEHFile *fptr = SD.FOpen("testRot3.txt","r");
    /* Open write files to track error and delta angular displacement */
    // This is useful for tuning among other things
    FEHFile *fOutErrptr = SD.FOpen("errorLog.txt","w");
    FEHFile *fOutDispptr = SD.FOpen("dispLog.txt","w");
    
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
    // Should also remove this
    LCD.Clear(FEHLCD::Blue);
    // Should remove this later
    if(i < 50) {
        LCD.Clear(FEHLCD::Black);
    }
    /* Close trajectory file */
    SD.FClose(fptr);
    /* PI LOOP */
    // Yes, not PID as the derivative term isn't needed currently
    for (int i = 0; i < size; i++) {
        /* Get new encoder counts */
        countNew1 = motor1_encoder.Counts();
        countNew2 = motor2_encoder.Counts();
        countNew3 = motor3_encoder.Counts();
        /* Check if first iteration (no old values) */
        // This is redundant! (remove later)
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
        if(vel_ref[0][i] < 0.0) {
            motorSpeed1 *= -1.0;
        }
        if(vel_ref[1][i] < 0.0) {
            motorSpeed2 *= -1.0;
        }
        if(vel_ref[2][i] < 0.0) {
            motorSpeed3 *= -1.0;
        }

        //motor1.SetPercent(limitMotorPercent(motorSpeed1));
        //motor2.SetPercent(limitMotorPercent(motorSpeed2));
        //motor3.SetPercent(limitMotorPercent(motorSpeed3));
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

int getCdsColor() {
    // Red
    if(CdS_cell.Value() < 1 ) {
        LCD.Clear(RED);
        return 2;
    // Blue
    } else if(CdS_cell.Value() >= 1 && CdS_cell.Value() < 1.8 ) {
        LCD.Clear(BLUE);
        return 1;
    } else {
        LCD.Clear(BLACK);
        return 0;
    }
}

void startMoveToJukeBoxAndRamp() {
    moveForward(25, 18);
    allStop();
    // Read Cds Cell
    int cdsValue = getCdsColor();
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

int main(void)
{

    jukebox_servo.SetMin(700);
    jukebox_servo.SetMax(2380);
    arm_servo.SetMin(508);
    arm_servo.SetMax(2464);
    jukebox_servo.SetDegree(5.0);
    motor1_encoder.ResetCounts();
    motor2_encoder.ResetCounts();
    motor3_encoder.ResetCounts();

    arm_servo.TouchCalibrate();
    /*
    float trash_x, trash_y;
    while(!LCD.Touch(&trash_x, &trash_y)) {
        LCD.WriteLine(CdS_cell.Value());
    }
    //while(getCdsColor() != 2);
    //PIDMoveTo("posRefTestRotate3.txt", 31);
    */



    return 0;
}
