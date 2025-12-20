#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/intake.h"
#include "../custom/include/logger.h"
#include "../../include/driveSettings.h"


ChassisGeometry chassisGeometry(&left_chassis, &right_chassis, 1);
ChassisDriverSettings chassisDriverSettings(&controller_1, 1, 0, 10, false);
TwoStickArcade chassis(chassisGeometry, chassisDriverSettings);

// Modify autonomous, driver, or pre-auton code below
int auton_selected = 0;
bool auto_started = false;
vex::thread* odom = nullptr;

void taskHandler(bool driver){
  if(!driver && odom == nullptr){
    odom = new vex::thread(trackNoOdomWheel);
  } 
  else if (driver && odom != nullptr) {
    odom->interrupt();
    delete odom;
    odom = nullptr;
  
}

}
void runAutonomous() {
  auto_started = true;
  switch(auton_selected) {
    case 0:
      awp();
      break;
    case 1:
      left7LongandWing();
      break;  
    case 2:
      right7LongandWing();
      break;
    case 3:
      leftLongAndMid();
      break; 
    case 4:
      leftLongAndMidDisrupt();
      break;
    case 5:
      left4();
      break;
    case 6:
      rifour();
      break;
    case 7:
      //newChangeQOL();
      break;

    
  }
  
}

int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;
//Logger logger(std::cout, Logger::Level::DEBUG);

void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  bool downPressed;
  bool bPressed;
  bool upPressed;
  taskHandler(false);
  
  matchloader.set(false);
  while (true) {
    uint64_t timestamp = vex::timer::systemHighResolution();
    Brain.Screen.clearScreen(black);
    antiJamTask();
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();
    
    // default tank drive or replace it with your preferred driver code here: 
    driveChassis(ch3 * 0.12 + ch1 * 0.123, ch3 * 0.12 - ch1 * 0.123);
    //chassis.controllerFeedbackSpin(false);
    
    if(r1){
      storeIntake();
      middleGoal.set(false);
    } else if(r2){
      scoreLongGoal();
      middleGoal.set(false);
    } else if(l1){
      middleGoal.set(true);
      scoreMiddleGoal();
    } else if(l2){
      leftWing.set(false);
      middleGoal.set(false);
      
    } else if(button_b){
      outtake();
    }
    else{
      stopIntake();
      leftWing.set(true);
      middleGoal.set(false);
    }
    if(controller_1.ButtonDown.PRESSED){
      downPressed = !downPressed;
      if(downPressed){
        matchloader.set(true);
      } else{
        matchloader.set(false);
      }
    }
    Brain.Screen.printAt(5, 60, "x: %.2f", x_pos);
    Brain.Screen.printAt(5, 80, "y: %.2f", y_pos);
    Brain.Screen.printAt(5, 100, "heading: %.2f", normalizeTarget(getInertialHeading()));
    Brain.Screen.printAt(5, 120, "battery: %.2f", vexBatteryCapacityGet);
    

    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
}


void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }
  controller_1.rumble("..--");
  
  // odom tracking
  resetChassis();
  taskHandler(false);
  
  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Selected Auton:");
    Brain.Screen.printAt(5, 40, "-----------");
    Brain.Screen.printAt(5, 90, "-----------");
 
    switch(auton_selected){
      case 0:
        Brain.Screen.printAt(5, 60, "AWP");
        break;
      case 1:
        Brain.Screen.printAt(5, 60, "Left 7 Wing");
        break;
      case 2:
        Brain.Screen.printAt(5, 60, "Right 7 Wing");
        break;
      case 3:
        Brain.Screen.printAt(5, 60, "Left Long And Mid Rush");
        break;
      case 4:
        Brain.Screen.printAt(5, 60, "Left Long And Mid No Rush");
        break;
      case 5:
        Brain.Screen.printAt(5, 60, "Left 4 Wing");
        break;
      case 6:
        Brain.Screen.printAt(5, 60, "Right 4 Wing");
        break;
      case 7:
        Brain.Screen.printAt(5, 60, "Test Auto");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      auton_selected ++;
    } else if (auton_selected == 7){
      auton_selected = 0;
    }
    wait(10, msec);
    
  }
}