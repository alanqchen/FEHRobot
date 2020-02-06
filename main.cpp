#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <math.h>
/*
typedef double real_t;

typedef struct {
    real_t derState; // Last position input
    real_t integratState; // Integrator state
    real_t integratMax, // Maximum and minimum
    integratMin; // allowable integrator state
    real_t integratGain, // integral gain
    propGain, // proportional gain
    derGain; // derivative gain
} SPid;

typedef struct {
    double currEnCount;
    double prevEnCount;
    double currPos;
    double prevPos;
    double currVel;
    double prevVel;
} PID;

typedef PID* pid_t;

real_t UpdatePID(SPid * pid, real_t error, real_t position) {
    real_t pTerm, dTerm, iTerm;
    pTerm = pid->propGain * error; // calculate the proportional term
    // calculate the integral state with appropriate limiting
    pid->integratState += error;
    // Limit the integrator state if necessary
    if (pid->integratState > pid->integratMax)
    {
    pid->integratState = pid->integratMax;
    }
    else if (pid->integratState < pid->integratMin)
    {
    pid->integratState = pid->integratMin;
    }
    // calculate the integral term
    iTerm = pid->integratGain * pid->integratState;
    // calculate the derivative
    dTerm = pid->derGain * (pid->derState - position);
    pid->derState = position;
    return pTerm + dTerm + iTerm;
}

void PIDtoPoint(int x, int y, int speedPercent, FEHMotor topMotor, FEHMotor leftMotor, FEHMotor rightMotor,
                DigitalEncoder topEncoder, DigitalEncoder leftEncoder, DigitalEncoder rightEncoder, double delta_t) {
    double t1;
    int disp1 = 0, disp2 = 0; disp3 = 0;
    double phil = 0.0, phi2 = 0.0, phi3 = 0.0;
    double phidot1 = 0.0, phidot2 = 0.0, phidot3 = 0.0;
    double u1 = 0.0, u2 = 0.0, u3 = 0.0;
    double e_new1 = 0.0, e_new2 = 0.0, e_new3 = 0.0;
    double e_total1 = 0.0, e_total2 = 0.0, e_total3 = 0.0;
    double Kp = 90.0;
    double Ki = 18.0;
    double Kd = 0.0;
    int FirstRun = true;
    double encoderRes = 318.0;
    // Populate structs
    struct PID pidMotorTop;
    struct PID pidMotorLeft;
    struct PID pidMotorRight;


    while(RT_Count <= NT) {
        t1 = rt_get_cpu_time_ns() / 1000000;
        pidMotorTop.currEnCount = read_encoder(1);
        pidMotorLeft.currEnCount = read_encoder(2);
        pidMotorRight.currEnCount = read_encoder(3);

        if(FirstRun) {
            disp1 = calc_disp(pidMotorTop.currEnCount, 0);
            disp2 = calc_disp(pidMotorLeft.currEnCount, 0);
            disp3 = calc_disp(pidMotorRight.currEnCount, 0);
            FirstRun = false;
        } else {
            disp1 = calc_disp(pidMotorTop.currEnCount, pidMotorTop.prevEnCount);
            disp2 = calc_disp(pidMotorLeft.currEnCount, pidMotorLeft.prevEnCount);
            disp3 = calc_disp(pidMotorRight.currEnCount, pidMotorRight.prevEnCount);
        }

        // Calculate angular position
        phi1 += disp1 * (2.0 * M_PI / encoderRes);
        phi2 += disp2 * (2.0 * M_PI / encoderRes);
        phi3 += disp3 * (2.0 * M_PI / encoderRes);

        pidMotorTop = phi1;
        pidMotorLeft = phi2;
        pidMotorRight = phi3;

        // Calculate displacement
        phidot1 = (disp1 * (2.0 * M_PI / encoderRes)) / delta_t;
        phidot2 = (disp2 * (2.0 * M_PI / encoderRes)) / delta_t;
        phidot3 = (disp3 * (2.0 * M_PI / encoderRes)) / delta_t;

        pidMotorTop.currVel = phidot1;
        pidMotorLeft.currVel = phidot2;
        pidMotorRight.currVel = phidot3;

        // Calculate errors
        if(u1 <= 100 && u1 >= 0)
            e_total1 += e_new1;
        if(u2 <= 100 && u2 >= 0)
            e_total2 += e_new2;
        if(u3 <= 100 && u3 >= 0)
            e_total3 += e_new3;
        u1 = Kp * e_new1 + Ki * delta_t * e_total1;
        u2 = Kp * e_new2 + Ki * delta_t * e_total2;
        u3 = Kp * e_new3 + Ki * delta_t * e_total3;

        topMotor.SetPercent((float)u1);
        leftMotor.SetPercent((float)u1);
        rightMotor.SetPercent((float)u1);


    }
}



void PIDGotoPoint(int x, int y, int speedPercent, FEHMotor topMotor, FEHMotor leftMotor, FEHMotor rightMotor,
                  DigitalEncoder topEncoder, DigitalEncoder leftEncoder, DigitalEncoder rightEncoder, double delta_t) {
    position = ReadPlantADC();
    drive = UpdatePID(&plantPID, plantCommand - position, position);
    DrivePlantDAC(drive);
}
*/
/*
int main()
{
    double t_now;
    t_now = TimeNow();
    // Init motors
    FEHMotor topMotor(FEHMotor::Motor0, 9.0);
    FEHMotor leftMotor(FEHMotor::Motor1, 9.0);
    FEHMotor rightMotor(FEHMotor::Motor2 ,9.0);

    // Init encoders
    DigitalEncoder topEncoder(FEHIO::P0_0);
    DigitalEncoder leftEncoder(FEHIO::P0_1);
    DigitalEncoder rightEncoder(FEHIO::P0_2);
    double delta_t = TimeNow() - t_now;
    // Init RPS
    RPS.InitializeTouchMenu();

    //PIDGotoPoint(20, 20, 20, topMotor, leftMotor, rightMotor, topEncoder, leftEncoder, rightEncoder, delta_t);

    return 0;
}
*/
