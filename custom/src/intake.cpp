#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

double intakeSpeed = 0;
double hoodSpeed = 0;
bool rejectBall = false;
bool isRed = false;

void manualIntake(double speed, double hoodSpd){
    intakeSpeed = speed;
    hoodSpeed = hoodSpd;
    intake.spin(forward, speed, voltageUnits::volt);
    hood.spin(forward, hoodSpd, voltageUnits::volt);
}
void stopIntake() {
    intakeSpeed = 0;
    hoodSpeed = 0;
    intake.spin(forward, 0, voltageUnits::volt);
    hood.spin(forward, 0, voltageUnits::volt);
}

void scoreLongGoal(){
    intakeSpeed = 12;
    hoodSpeed = 12;
    intake.spin(forward, 12, voltageUnits::volt);
    hood.spin(forward, 12, voltageUnits::volt);
}

void scoreMiddleGoal(){
    intakeSpeed = 12;
    hoodSpeed = 10;
    intake.spin(forward, 12, voltageUnits::volt);
    hood.spin(forward, 10, voltageUnits::volt);
}

/**
 * @brief Move the intake backwards at maximum speed to eject the ball.
 * This function is typically used to shoot the ball out of the intake.
 */
void outtake(){
    intakeSpeed = -12;
    hoodSpeed = -12;
    intake.spin(forward, -12, voltageUnits::volt);
    hood.spin(forward, -12, voltageUnits::volt);
}
void storeIntake(){
    intakeSpeed = 12;
    hoodSpeed = -3;
    intake.spin(forward, 12, voltageUnits::volt);
    hood.spin(forward, -3, voltageUnits::volt);
}
void antiJamTask(){
        if((intakeSpeed != 0 && fabs(intake.velocity(vex::velocityUnits::rpm)) < 5 && intake.torque(vex::torqueUnits::Nm) > 0.2)){
            intake.spin(forward, -intakeSpeed, voltageUnits::volt);
            vex::wait(100, msec);
            intake.spin(forward, intakeSpeed, voltageUnits::volt);
        } 
        

       
}