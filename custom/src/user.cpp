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
int auton_selected = 1;
int color_selected = 1;
bool auto_started = false;
vex::thread* odom = nullptr;


enum PreAutonState {
  SELECT_AUTON,
  SELECT_COLOR,
  SHOW_SELECTION,
  DONE
};

PreAutonState preAutonState = SELECT_AUTON;


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
  Brain.Screen.clearScreen();
  auto_started = true;
  switch(auton_selected) {
    case 0:
      awp(color_selected);
      break;
    case 1:
      left7LongandWing(color_selected);
      break;  
    case 2:
      right7LongandWing(color_selected);
      break;
    case 3:
      leftLongAndMid(color_selected);
      break; 
    case 4:
      leftLongAndMidDisrupt(color_selected);
      break;
    case 5:
      left4();
      break;
    case 6:
      rifour();
      break;
    case 7:
      matchloaderLeftFourBall(color_selected);
      break;
    case 8:
      matchloaderRightFourBall(color_selected);
      break;
    case 9:
      matchloaderLeftLongAndMid(color_selected);
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
  Brain.Screen.clearScreen();
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

    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
}


void runPreAutonomous() {
  vexcodeInit();

  inertial_sensor.calibrate();
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  controller_1.rumble("..--");

  resetChassis();
  taskHandler(false);

  ballSensTop.setLight(vex::ledState::on);
  ballSensTop.setLightPower(100, pct);

  while (!auto_started) {
    Brain.Screen.clearScreen();

    if (preAutonState == SELECT_AUTON) {
      Brain.Screen.printAt(5, 20, "Select Autonomous");
      Brain.Screen.printAt(5, 40, "Tap to cycle, Hold to confirm");
      Brain.Screen.printAt(5, 60, "---------------------");

      switch (auton_selected) {
        case 0: Brain.Screen.printAt(5, 90, "AWP"); break;
        case 1: Brain.Screen.printAt(5, 90, "Left 7 Wing"); break;
        case 2: Brain.Screen.printAt(5, 90, "Right 7 Wing"); break;
        case 3: Brain.Screen.printAt(5, 90, "Left Long + Mid Rush"); break;
        case 4: Brain.Screen.printAt(5, 90, "Left Long + Mid No Rush"); break;
        case 5: Brain.Screen.printAt(5, 90, "Left 4 Wing"); break;
        case 6: Brain.Screen.printAt(5, 90, "Right 4 Wing"); break;
        case 7: Brain.Screen.printAt(5, 90, "Matchloader Left 4 Wing"); break;
        case 8: Brain.Screen.printAt(5, 90, "Matchloader Right 4 Wing"); break;
        case 9: Brain.Screen.printAt(5, 90, "Matchloader Left Long + Mid"); break;
      }

      if (Brain.Screen.pressing()) {
        uint32_t pressTime = Brain.timer(msec);
        while (Brain.Screen.pressing()) {}

        if (Brain.timer(msec) - pressTime > 500) {
          preAutonState = SELECT_COLOR; // long press = confirm
        } else {
          auton_selected = (auton_selected + 1) % 10; // short tap = cycle
        }
      }
    }

    else if (preAutonState == SELECT_COLOR) {
      Brain.Screen.printAt(5, 20, "Select Alliance Color");
      Brain.Screen.printAt(5, 40, "Tap to cycle, Hold to confirm");
      Brain.Screen.printAt(5, 60, "---------------------");

      if (color_selected == 0)
        Brain.Screen.printAt(5, 90, "RED");
      else
        Brain.Screen.printAt(5, 90, "BLUE");

      if (Brain.Screen.pressing()) {
        uint32_t pressTime = Brain.timer(msec);
        while (Brain.Screen.pressing()) {}

        if (Brain.timer(msec) - pressTime > 500) {
          preAutonState = SHOW_SELECTION;
        } else {
          color_selected = !color_selected;
        }
      }
    }
    
    else if (preAutonState == SHOW_SELECTION) {
      Brain.Screen.printAt(5, 20, "AUTO & COLOR SELECTED");
      Brain.Screen.printAt(5, 40, "---------------------");
    
      Brain.Screen.printAt(5, 70, "Autonomous:");
    
      switch (auton_selected) {
        case 0: Brain.Screen.printAt(20, 90, "AWP"); break;
        case 1: Brain.Screen.printAt(20, 90, "Left 7 Wing"); break;
        case 2: Brain.Screen.printAt(20, 90, "Right 7 Wing"); break;
        case 3: Brain.Screen.printAt(20, 90, "Left Long + Mid Rush"); break;
        case 4: Brain.Screen.printAt(20, 90, "Left Long + Mid No Rush"); break;
        case 5: Brain.Screen.printAt(20, 90, "Left 4 Wing"); break;
        case 6: Brain.Screen.printAt(20, 90, "Right 4 Wing"); break;
        case 7: Brain.Screen.printAt(20, 90, "Matchloader Left 4 Wing"); break;
        case 8: Brain.Screen.printAt(20, 90, "Matchloader Right 4 Wing"); break;
        case 9: Brain.Screen.printAt(20, 90, "Matchloader Left Long + Mid"); break;
      }
    
      Brain.Screen.printAt(5, 120, "Color:");
      if (color_selected == 0)
        Brain.Screen.printAt(20, 140, "RED");
      else
        Brain.Screen.printAt(20, 140, "BLUE");
    
      Brain.Screen.printAt(5, 180, "Hold screen to finish");
    
      if (Brain.Screen.pressing()) {
        uint32_t pressTime = Brain.timer(msec);
        while (Brain.Screen.pressing()) {}
    
        if (Brain.timer(msec) - pressTime > 500) {
          preAutonState = DONE;
        }
      }
    }

    else if(preAutonState == DONE){
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(5, 20, "Odom:");
      Brain.Screen.printAt(20, 40, "x: %.2f", x_pos);
      Brain.Screen.printAt(20, 60, "y: %.2f", y_pos);
      Brain.Screen.printAt(20, 80, "heading: %.2f", inertial_sensor.heading(deg));
      Brain.Screen.printAt(5, 100, "Telemetry:");
      Brain.Screen.printAt(20, 120, "front distance sensor(in): %.2f", frontDistanceSensor.value()/25.4);
      Brain.Screen.printAt(20, 140, "left distance sensor(in): %.2f", leftDistanceSensor.value()/25.4);
      Brain.Screen.printAt(20, 160, "right distance sensor(in): %.2f", rightDistanceSensor.value()/25.4);
      Brain.Screen.printAt(20, 180, "hue: %.2f", ballSensTop.hue());
      Brain.Screen.printAt(20, 200, "battery: %.2f", vexBatteryCapacityGet);

    }

    wait(20, msec);
  }
  Brain.Screen.clearScreen();
  
  // ----- CLEANUP AFTER PRE-AUTON -----
  
}

