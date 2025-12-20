#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>
#include "pose.h"
#include "../include/autonomous.h"
#include "motor-control.h"
#include "../custom/include/intake.h"
#include "../include/maths.h"
#include "../../genFunctions/include/main.h"
#include "../../genFunctions/include/motionControl/PID.h"
#include "../../genFunctions/include/motionControl/Odometry.h"
#include "../../genFunctions/include/motionControl/Odom.h"
#include "../../genFunctions/include/motionControl/Motion.h"
#include "../../genFunctions/include/utility/Path.h"
#include "../../genFunctions/include/PathStorage/testPath.h"
#include "../../genFunctions/include/components/Inertial.h"

Odometry odometry(-0.09, 0.5, 1.1);


Inertial imu(inertial_sensor, 1.00035);



AsymptoticGains lateralKp1 = AsymptoticGains(15000, 15000, 1, 1);
AsymptoticGains angularKp1 = AsymptoticGains(880, 400, 28, 1.7);
AsymptoticGains correctKp1 = AsymptoticGains(200, 200, 1, 1);

MotionPID langPID = MotionPID(lateralKp1, 25000, 150, 0.5, true);
MotionPID angPID = MotionPID(angularKp1, 2000, 12, 5, true);
MotionPID headingPID = MotionPID(correctKp1, 0, 40, 0, false);

Motion motion(
  langPID,
  angPID,
  headingPID,
  odometry);


double PHI_FRONT = 0.0;
double PHI_RIGHT = 90.0;
double PHI_LEFT  = -90.0;
// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }
//s
//new change examples
void resetOdometry(double x, double y){
  x_pos = x;
  y_pos = y;
}

double readSensor(vex::distance &s) {
  return s.value() / 25.4; // mm → inches
}

double getWallCoordinate(char wall) {
  switch (wall) {
      case 'N': return 72; // North wall is +Y max
      case 'S': return -72;         // South wall is Y = -72
      case 'E': return 72; // East wall is +X max
      case 'W': return -72;         // West wall is X = 0
  }
  return 0.0;
}

double computeProjection(double d, double thetaDeg, double phiDeg, bool projectX) {
  double the = degToRad(thetaDeg + phiDeg);

  if (projectX)
      return d * cos(the);
  else
      return d * sin(the);
}

void relocalize(std::string walls) {
  LegacyPose p = {NAN, NAN, NAN};
  double headingDeg = normalizeTarget(getInertialHeading());

  bool useNorth = walls.find('N') != std::string::npos;
  bool useSouth = walls.find('S') != std::string::npos;
  bool useEast  = walls.find('E') != std::string::npos;
  bool useWest  = walls.find('W') != std::string::npos;

  double dFront = readSensor(frontDistanceSensor);
  double dRight = readSensor(rightDistanceSensor);
  double dLeft  = readSensor(leftDistanceSensor);

  // ---------- Y COORDINATE (North/South wall) ----------
  if (useNorth || useSouth) {
      char wall = useNorth ? 'N' : 'S';
      double Ywall = getWallCoordinate(wall);

      // Determine which sensor points toward the wall
      // North wall = +Y direction
      // South wall = -Y direction
      double bestDist = INFINITY;
      double sensorDist, phi;

      if (useNorth) {
          // sensor direction . global +Y
          sensorDist = dFront; phi = PHI_FRONT;
      }
      else { // South (negative Y)
          sensorDist = dFront; phi = PHI_FRONT;
          
      }

      double projY = computeProjection(sensorDist, headingDeg, phi, false);
      if (useSouth) projY = -projY;

      p.y = Ywall - projY;
  }


  // ---------- X COORDINATE (East/West wall) ----------
  if (useEast || useWest) {
      char wall = useEast ? 'E' : 'W';
      double Xwall = getWallCoordinate(wall);

      double bestDist = INFINITY;
      double sensorDist, phi;

      if (useEast) { // +X
          sensorDist = dRight; phi = PHI_RIGHT; bestDist = sensorDist * cos(degToRad(headingDeg + phi));

          double projF = dFront * cos(degToRad(headingDeg + PHI_FRONT));
          if (projF > 0 && projF < bestDist) { sensorDist = dFront; phi = PHI_FRONT; bestDist = projF; }

          double projL = dLeft * cos(degToRad(headingDeg + PHI_LEFT));
          if (projL > 0 && projL < bestDist) { sensorDist = dLeft; phi = PHI_LEFT; bestDist = projL; }
      }
      else { // West, -X
          sensorDist = dLeft; phi = PHI_LEFT; bestDist = -sensorDist * cos(degToRad(headingDeg + phi));

          double projF = -dFront * cos(degToRad(headingDeg + PHI_FRONT));
          if (projF > 0 && projF < bestDist) { sensorDist = dFront; phi = PHI_FRONT; bestDist = projF; }

          double projR = -dRight * cos(degToRad(headingDeg + PHI_RIGHT));
          if (projR > 0 && projR < bestDist) { sensorDist = dRight; phi = PHI_RIGHT; bestDist = projR; }
      }

      double projX = computeProjection(sensorDist, headingDeg, phi, true);
      if (useWest) projX = -projX;

      p.x = Xwall - projX;
  }
  resetOdometry(p.x, p.y);
}

void newChangeQOL(){
  followPath(Point(-10.5, 24), Point(-10.5, 24), Point(-10.5, 24), Point(-10.5, 24), true, 2000);
  relocalize("NW");
  motion.movePose(10, 15, 90, 0.7, 0, 60, 100, 2000, -1, false);
  motion.turnHeading(60, 0, 1, 1, 600);
}

void left9Long(){
  
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-10.5, 24, 1, 2000, false, 5);
  matchloader.set(false);
  vex::task matchloadDeploy2([]{
    vex::wait(950, msec);
    // matchloader.set(true);
    return 0;
  });
  boomerang(-31, 32, 1, -39, 0.1, 3000, true, 6);
  vex::wait(80, msec);
  heading_correction_kp = 0.3;
  moveToPoint(-23, 10, -1, 3000, false, 8);
  stopIntake();
  moveToPoint(-38, 8, -1,  3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveToHeading(-16.8, 180, 3000, true, 7);
  driveChassis(-1,-1);
  scoreLongGoal();
  vex::wait(1300, msec);
  driveChassis(0,0);
  resetOdom(-31.5, 15);
  matchloader.set(true);
  max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
dir_change_end = true;   
  moveToPoint(-28.5, -9, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(1380, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-29, 10, -1, 3000, false, 6);
  driveChassis(-6, -6);
  vex::wait(1200, msec);
  driveChassis(0,0);
  resetChassis();
  driveChassis(4,4);
  vex::wait(500, msec);
  driveChassis(-6,-6);
  vex::wait(700, msec);
  driveChassis(0,0);
}

void left9LongDisrupt(){
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(800, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-7, 24, 1, 2000, false, 6);
  boomerang(-30, 33.5, 1, -39, 0.1, 3000, true, 9);
  matchloader.set(true);
  vex::wait(80, msec);
  heading_correction_kp = 0.3;
  moveToPoint(-23, 15, -1, 3000, false, 8);
  boomerang(-32, 15, -1,90, 0.1, 3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveToHeading(-14, 180, 3000, true, 7);
  scoreLongGoal();
  vex::wait(1100, msec);
  driveChassis(0,0);
  resetOdom(-31.5, 15);
  matchloader.set(true);
  moveToPoint(-30, -7, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(950, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-31, 10, -1, 3000, false, 6);
  driveChassis(-6, -6);
  vex::wait(1000, msec);
  driveToHeading(12, 180, 3000, true, 6);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 6);
}


void right9Long(){
  leftWing.set(true);
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = 45;
  vex::task matchloadDeploy([]{
    vex::wait(800, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(9, 24, 1, 2000, false, 5);
  matchloader.set(false);
  vex::task matchloadDeploy2([]{
    vex::wait(1000, msec);
    matchloader.set(true);
    return 0;
  });
  boomerang(34, 43, 1, 40, 0.1, 3000, true, 5);
  driveChassis(2,2);
  vex::wait(0.1, sec);
  matchloader.set(true);
  vex::wait(80, msec);
  //moveToPoint(34, 30, -1, 3000, false, 8);
  moveToPoint(26, 13, -1, 3000, false, 8);
  stopIntake();
  moveToPoint(47.8,10, -1, 3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveChassis(-6,-6);
  vex::wait(0.4, sec);
  driveChassis(-1, -1);
  scoreLongGoal();
  vex::wait(1500, msec);
  matchloader.set(true);
  moveToPoint(49, 6, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(950, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(49, 10, -1, 3000, false, 6);
  driveToHeading(-16, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(1500, msec);
  driveToHeading(12, 180, 3000, true, 6);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 6);

}

void right9LongDisrupt(){
  leftWing.set(true);
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = 45;
  vex::task matchloadDeploy([]{
    vex::wait(800, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(6, 24, 1, 2000, false, 5);
  matchloader.set(true);
  vex::task matchloadDeploy2([]{
    vex::wait(1000, msec);
    matchloader.set(true);
    return 0;
  });
  boomerang(32, 45, 1, 40, 0.1, 3000, true, 12);
  matchloader.set(true);
  vex::wait(80, msec);
  heading_correction_kp = 0.3;
  moveToPoint(28, 13, -1, 3000, false, 8);
  stopIntake();
  boomerang(48, 13, -1,-90, 0.1, 3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveToHeading(-14, 180, 3000, true, 7);
  scoreLongGoal();
  vex::wait(1500, msec);
  matchloader.set(true);
  moveToPoint(48.5, 6, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(1000, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(49, 10, -1, 3000, false, 6);
  driveToHeading(-20, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(1500, msec);
  driveToHeading(12, 180, 3000, true, 6);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 6);

}

void leftLongAndMid(){
  leftWing.set(true);
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-8.8, 24.5, 1, 2000, false, 6);
  matchloader.set(false);
  vex::task matchloadDeploy2([]{
    vex::wait(850, msec);
    //matchloader.set(true);
    return 0;
  });
  correct_angle = -40;
  moveToPoint(-19, 31.5, 1, 3000, true, 6);
  
  driveChassis(1,1);
  vex::wait(300,msec);
  moveToPoint(-6, 23, -1, 3000, false, 5);
  //matchloader.set(false);
  turnToAngle(-138, 1000, true, 7);
  vex::task middleready([]{
    outtake();
   vex::wait(250, msec);
     stopIntake();
    matchloader.set(true);
   return 0;
  });
  driveTo(-11,2000, true, 5);
  middleGoal.set(true);
  scoreMiddleGoal();
  vex::wait(750, msec);
  middleGoal.set(false);
  stopIntake();
  middleGoal.set(false);
  matchloader.set(true);
  matchloader.set(true);
  max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
dir_change_end = true;   
  moveToPoint(-26.5, 0, 1, 3000, true, 8);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(1180, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-25, 5, -1, 3000, false, 8);
  driveToHeading(-20, 180, 1000, true, 7);
  scoreLongGoal();
  vex::wait(1200, msec);
  curveCircle(120, -16, 1000, false, 9);
  driveTo(2.5, 3000, false, 10);
  vex::task wingdep([]{
    vex::wait(350, msec);
    leftWing.set(false);
    return 0;
  });
  turnToAngle(180, 800, true, 7);
  driveTo(-31, 3000, true,4.8);
  turnToAngle(-160, 900, true, 10);
 
  stopChassis(brakeType::hold);

}
void awp2(){
  min_output = 100;
 max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
dir_change_end = true;  
  leftWing.set(true);
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-7, 24, 1, 2000, true, 5);
  matchloader.set(false);
  vex::task matchloadDeploy2([]{
    vex::wait(850, msec);
    //matchloader.set(true);
    return 0;
  });
  boomerang(-24, 33, 1, -44, 0.1, 3000, false, 6);
  driveChassis(4,4);
  vex::wait(100,msec);
  //matchloader.set(true);
  vex::wait(80, msec);
  moveToPoint(-0.9, 23, -1, 3000, false, 5);
  driveChassis(-1,-1);
  vex::wait(10, msec);
  //matchloader.set(false);
  turnToAngle(-138, 1000, true, 5);
  driveTo(-4,2000, false, 5);
  middleGoal.set(true);
  scoreMiddleGoal();
  vex::wait(400, msec);
  stopIntake();
  middleGoal.set(false);
  matchloader.set(true);
  moveToPoint(-22.8, 1, 1, 3000, false, 8);
  matchloader.set(true);
  //turnToAngle(180, 3000);
  //moveToPoint(-23.5, 0, 1, 3000, false, 6);
  storeIntake();
  //turnToAngle(180, 3000);


  driveToHeading(15,180,1500,false,6);
  vex::wait(500,msec);
  moveToPoint(-23, 5, -1, 3000, false, 10);
  driveToHeading(-11, 180, 3000, false, 6);
  scoreLongGoal();
  vex::wait(650, msec);
  driveToHeading(3, 180, 3000, false, 8);
  matchloader.set(false);

  heading_correction_kp=0.3;
  moveToPoint(36.5,30.5,1,6000,false,10);
  storeIntake();
  vex::task matchloadDeploy3([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  turnToAngle(90, 500, false, 8);

  //correct_angle = 160;
  moveToPoint(68,3,1,9000,false,9);
  turnToAngle(180, 600, true, 8);
  //turnToAngle(180,1000,true,6);
  matchloader.set(true);
  driveToHeading(7,180,3000,false,6);
  vex::wait(600,msec);
  moveToPoint(70.5,3,-1,6000,false,9);
  driveToHeading(-12,180,3000,false,6);
  scoreLongGoal();

  

}

void leftLongAndMidDisrupt(){
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-7, 24, 1, 2000, false, 5);
  matchloader.set(true);
  vex::task matchloadDeploy2([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  boomerang(-30, 34, 1, -39, 0.1, 3000, true, 12);
  vex::wait(80, msec);
  moveToPoint(-0.5, 23, -1, 3000, false, 5);
  driveChassis(-1,-1);
  vex::wait(10, msec);
  turnToAngle(-138, 1000, true, 5);
  driveTo(-7,2000, true, 5);
  middleGoal.set(true);
  scoreMiddleGoal();
  vex::wait(450, msec);
  stopIntake();
  middleGoal.set(false);
  moveToPoint(-23.6, -2, 1, 3000, false, 6);
  turnToAngle(178, 300, true, 6);
  driveToHeading(-18, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(850, msec);
  matchloader.set(true);
  moveToPoint(-23.5, -7, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(7,7);
  vex::wait(900, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-23, 5, -1, 3000, false, 6);
  driveToHeading(-10, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(1500, msec);
  driveToHeading(10, 180, 3000, true, 6);
  driveToHeading(-20, 180, 3000, true, 6);

}

void rightLongAndLow(){
  leftWing.set(true);
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  moveToPointChain(24, 24, 1, 2000, 9);
  moveToPoint(-24, 15, 1, 2000, true, 7);


}

void awp(){
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  //awp
  driveTo(5, 600, true, 8);
  resetOdometry(0,5);
  driveTo(-32, 2000, false, 10);
  driveTo(-3, 600, true, 5);

  matchloader.set(true);
  //moveToPoint(-0.2, -34, -1, 2000, true, 7);
  turnToAngle(-90, 800, true, 10);
  driveChassis(5.5, 5.5);
  vex::wait(600, msec);
  driveChassis(2, 2);
  vex::wait(300, msec);
  resetOdometry(-72 + frontDistanceSensor.value()/25.4, -72 + leftDistanceSensor.value()/25.4);
  vex::wait(10, msec);
  moveToPoint(-42, -54.7, -1, 2000, false, 10);
  driveToHeading(-10, -90, 600, true, 7);
  scoreLongGoal();
  vex::wait(500, msec);
  matchloader.set(false);
  vex::wait(880, msec);
  driveChassis(1,1);
  vex::wait(20, msec);
  turnToAngle(0, 3000, true, 12);
  resetOdometry(-72 + leftDistanceSensor.value()/25.4, y_pos);
  storeIntake();
  vex::wait(10, msec);
  driveChassis(12,12);
  vex::wait(50, msec);
  moveToPoint(-30, -35, 1, 2000, false, 10);
  matchloader.set(true);
  correct_angle = 0;
  vex::task offm([]{
    vex::wait(200, msec);
    matchloader.set(false);
    return 0;
  });
  moveToPoint(-31, -11, 1, 2000, false, 10);
  moveToPoint(-32, 17.5, 1, 2000, true, 7);
  matchloader.set(true);
  boomerang(-16, -2, -1, -40, 0.6, 1000, true, 7);
  driveChassis(-2, -2);
  vex::wait(50, msec);
  scoreMiddleGoal();
  vex::wait(700, msec);
  stopIntake();
  middleGoal.set(false);
  boomerang(-53, 29.5, 1, -50, 0.3, 3000, true, 10);
  storeIntake();
  turnToAngle(-90, 900, true, 10);
  resetOdometry(-72 + frontDistanceSensor.value()/25.4, 72 - rightDistanceSensor.value()/25.4);
  vex::wait(10, msec);
  driveChassis(5.5, 5.5);
  vex::wait(600, msec);
  driveChassis(2, 2);
  vex::wait(200, msec);
  resetOdometry(-72 + frontDistanceSensor.value()/25.4, 72 - rightDistanceSensor.value()/25.4);
  vex::wait(10, msec);
  vex::task scoreLong2([]{
    vex::wait(900, msec);
    scoreLongGoal();
    return 0;
  });
  driveToHeading(-40, -92, 3000, true, 10);
}

//todo
void left7LongandWing(){
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(600, msec);
    matchloader.set(true);
    return 0;
  });
  max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
  //goes to stack
  moveToPoint(-9, 23, 1, 2000, false, 8);
  matchloader.set(false);
  max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
 turnToAngle(-160, 800, false, 8);
  correct_angle = normalizeTarget(-160);
  max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
  moveToPoint(-23, -10, 1, 2000, false, 9);
  turnToAngle(180, 800, true, 8);
  driveChassis(5,5);
  vex::wait(0.8, sec);
  moveToPoint(-22.5, 5, -1, 2000, false, 8);
  turnToAngle(180, 800, true, 7);
  driveChassis(-7,-7);
  vex::wait(0.4, sec);
  
  scoreLongGoal();
  driveChassis(0,0);
  vex::wait(2, sec);
  stopIntake();
  resetOdometry(-21.5, 5);
  leftWing.set(true);
  //curveCircle(120, -14, 1000, false, 8);
  turnToAngle(90, 3000, false, 12);
  driveTo(3, 3000, false, 12);
  vex::task wingdep([]{
    vex::wait(350, msec);
    leftWing.set(false);
    return 0;
  });
  turnToAngle(180, 800, true, 7);
  driveTo(-31, 3000, true,4);
  turnToAngle(-160, 900, true, 10);
 
  stopChassis(brakeType::hold);
  //

}

void right7LongandWing(){
  heading_correction_kp = 0.8;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = 45;
  vex::task matchloadDeploy([]{ 
    vex::wait(600, msec);
    matchloader.set(true);
    return 0;
  });
  //goes to stack
  moveToPoint(9.4, 24, 1, 2000, false, 8);
  max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
dir_change_end = true;
  turnToAngle(140, 300, false, 7);
  heading_correction_kp = 0.67;
  correct_angle = normalizeTarget(160);
  //moveToPoint(31, 10, 1, 2000, false, 12);
  moveToPoint(42.6, -1, 1, 2000, false, 9);
  turnToAngle(-180, 800, true, 7);
  driveChassis(6,6);
  vex::wait(0.95, sec);
  moveToPoint(43.6, 5, -1, 2000, false, 8);
  turnToAngle(-180, 800, true, 7);
  driveChassis(-8,-8);
  vex::wait(0.35, sec);
  driveChassis(-1,-1);
  scoreLongGoal();
  driveChassis(0,0);
  vex::wait(2, sec);
  stopIntake();
  leftWing.set(true);
  curveCircle(120, -14, 1000, false, 9);
  driveTo(2.25, 3000, false, 10);
  
  vex::task wingdep([]{
    vex::wait(150, msec);
    leftWing.set(false);
    return 0;
  });
  turnToAngle(-180, 800, true, 7);
  driveTo(-31.5, 20000, true,4);
  turnToAngle(-165, 900, true, 10);
  stopChassis(brakeType::hold);
  //
}
void left4(){
  

  
  heading_correction_kp = 0.8;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(600, msec);
    matchloader.set(true);
    return 0;
  });
  //goes to stack
  moveToPoint(-9.3, 24, 1, 2000, false, 8);
  turnToAngle(-150, 300, false, 7);
  heading_correction_kp = 0.67;
  correct_angle = normalizeTarget(-160);
  max_slew_accel_fwd = 24;
max_slew_decel_fwd = 24;
max_slew_accel_rev = 24;
 max_slew_decel_rev = 24;
dir_change_end = true;
  moveToPoint(-23, -4, 1, 2000, true, 9);
  turnToAngle(180, 800, true, 7);
  driveToHeading(-17, 180, 1000, true, 8);
  scoreLongGoal();
  vex::wait(1.5, sec);
  stopIntake();
  curveCircle(120, -14, 1000, false, 9);
  driveTo(1.9, 3000, false, 10);
  vex::task wingdep([]{
    vex::wait(350, msec);
    leftWing.set(false);
    return 0;
  });
  turnToAngle(180, 800, true, 7);
  matchloader.set(false);
  driveTo(-29, 3000, true,4.6);
  turnToAngle(-160, 900, true, 10);
  stopChassis(brakeType::hold);
}
//   heading_correction_kp = 0.8;
//   vex::task antiJamF([]{
//     while(1){
//       antiJamTask();
//       vex::wait(20, msec);
//     }
//     return 0;
//   });
//   // Use this for tuning linear and turn pid
//   storeIntake();
//   correct_angle = -45;
//   vex::task matchloadDeploy([]{
//     vex::wait(600, msec);
//     matchloader.set(true);
//     return 0;
//   });
//   //goes to stack
//   moveToPoint(-9.3, 24, 1, 2000, false, 8);
//   max_slew_accel_fwd = 24;
// max_slew_decel_fwd = 24;
// max_slew_accel_rev = 24;
//  max_slew_decel_rev = 24;
// dir_change_end = true;
//   turnToAngle(50, 300, false, 7);
//   heading_correction_kp = 0.67;
  
  //
void rifour(){
  heading_correction_kp = 0.8;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = 45;
  vex::task matchloadDeploy([]{ 
    vex::wait(600, msec);
    matchloader.set(true);
    return 0;
  });
  //goes to stack
  moveToPoint(9.4, 24, 1, 2000, false, 8);
  turnToAngle(140, 300, false, 7);
  heading_correction_kp = 0.67;
  correct_angle = normalizeTarget(165);
  matchloader.set(false);
  //moveToPoint(31, 10, 1, 2000, false, 12);
  heading_correction_kp = 0.8;
  moveToPoint(39, 5, 1, 2000, false, 8);
  turnToAngle(180,1000,true,6);
  driveToHeading(-27,180,1000,true,6);
  scoreLongGoal();
  driveChassis(0,0);
  vex::wait(1.85, sec);
  heading_correction_kp = 1.1;

  leftWing.set(true);
  curveCircle(120, -14, 1000, false, 9);
  driveTo(2.5, 3000, false, 10);
  vex::task wingdep([]{
    vex::wait(350, msec);
    leftWing.set(false);
    return 0;
  });
  turnToAngle(180, 800, true, 7);
  driveTo(-31, 3000, true,4);
  turnToAngle(-160, 900, true, 10);
 
  stopChassis(brakeType::hold);
}