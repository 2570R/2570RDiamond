#include "motionControl/Motion.h"



void Motion::setUnit(float unit) {
  //changes the length unit of the motion algorithms

  this->unit = unit;
}


void Motion::setPursuitSettings(float minLookahead, float maxLookahead, float pursuitVeloConst, float pursuitCurvConst, float pursuitCTEConst, float radiusLookahead) {
  //changes pure pursuit dynamic lookahead settings

  this->minLookahead = minLookahead;
  this->maxLookahead = maxLookahead;
  this->pursuitVeloConst = pursuitVeloConst;
  this->pursuitCurvConst = pursuitCurvConst;
  this->pursuitCTEConst = pursuitCTEConst;
  this->radiusLookahead = radiusLookahead;
}


void Motion::moveDist(float target, float maxSpeed, float timeout) {
  //moves forward for target distance

  int timer = 0;

  float error;

  lateral.reset(target);
  lateral.setKp(target);

  headingCorrect.reset(Misc::angleError(headingTarget, Misc::getHeading(false)));
  headingCorrect.setKp(Misc::angleError(headingTarget, Misc::getHeading(false)));

  target += Units::tickToUnit(vertical_tracker.position(vex::deg), 2, foot);

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    error = target - Units::tickToUnit(vertical_tracker.position(vex::deg), 2, foot);
    
    float lateralOutput = Misc::clamp(lateral.tick(error), -fabs(maxSpeed) * 12000.0, fabs(maxSpeed) * 12000.0);
    
    float headingOutput = Misc::clamp(headingCorrect.tick(Misc::angleError(headingTarget, Misc::getHeading(false))), -fabs(maxSpeed) * 12000.0, fabs(maxSpeed) * 12000.0);
    
    std::pair<float, float> outputs = Misc::reduceRatio(fabs(maxSpeed) * 12000.0, lateralOutput + headingOutput, lateralOutput - headingOutput);
    Misc::runDrivetrain(outputs.first, outputs.second, false);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (( Exit::error(error, 0.05)
        || Exit::velo(odometry.getSpeed(), 0.5))
        && Exit::time(timer, timeout));

  left_chassis.stop();
  right_chassis.stop();
}


void Motion::push(float speed, float timeout) {
  //pushes until cannot move anymore
  
  int timer = 0;
  
  headingCorrect.reset(Misc::angleError(headingTarget, Misc::getHeading(false)));
  headingCorrect.setKp(Misc::angleError(headingTarget, Misc::getHeading(false)));

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    float lateralOutput = speed * 12000.0;

    float headingOutput = headingCorrect.tick(Misc::angleError(headingTarget, Misc::getHeading(false)));

    std::pair<float, float> outputs = Misc::reduceRatio(fabs(speed) * 12000.0, lateralOutput + headingOutput, lateralOutput - headingOutput);
    Misc::runDrivetrain(outputs.first, outputs.second, false);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while ( (Exit::velo(odometry.getSpeed(), 0.05) && fabs(inertial_sensor.roll(vex::deg)) < 10)
        && Exit::time(timer, timeout));

  left_chassis.stop();
  right_chassis.stop();
}


void Motion::push(float coord, bool isY, float speed, float timeout) {
  //pushes until reaches coordinate and cannot move anymore
  
  int timer = 0;

  bool start;
    (isY) ? start = odometry.getPosY() > coord : odometry.getPosX() > coord;
  bool cur;
  
  headingCorrect.reset(Misc::angleError(headingTarget, Misc::getHeading(false)));
  headingCorrect.setKp(Misc::angleError(headingTarget, Misc::getHeading(false)));

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    cur = (isY) ? odometry.getPosY() > coord : odometry.getPosX() > coord;

    float lateralOutput = speed * 12000.0;

    float headingOutput = headingCorrect.tick(Misc::angleError(headingTarget, Misc::getHeading(false)));

    std::pair<float, float> outputs = Misc::reduceRatio(fabs(speed) * 12000.0, lateralOutput + headingOutput, lateralOutput - headingOutput);
    Misc::runDrivetrain(outputs.first, outputs.second, false);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (( (Exit::velo(odometry.getSpeed(), 0.5) && fabs(inertial_sensor.roll(vex::deg)) < 10)
        || start == cur)
        && Exit::time(timer, timeout));

  left_chassis.stop();
  right_chassis.stop();
}


void Motion::crossBarrier() {
  //crosses barrier

  bool reachedMinPitch = false;
  bool reachedMaxPitch = false;
  
  headingCorrect.reset(Misc::angleError(headingTarget, Misc::getHeading(false)));
  headingCorrect.setKp(Misc::angleError(headingTarget, Misc::getHeading(false)));

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    if (inertial_sensor.roll(vex::deg) < -10) reachedMinPitch = true;
    if (inertial_sensor.roll(vex::deg) > 10) reachedMaxPitch = true;

    float lateralOutput = 12000.0;

    float headingOutput = headingCorrect.tick(Misc::angleError(headingTarget, Misc::getHeading(false)));

    std::pair<float, float> outputs = Misc::reduceRatio(12000.0, lateralOutput + headingOutput, lateralOutput - headingOutput);
    Misc::runDrivetrain(outputs.first, outputs.second, false);

    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (!reachedMinPitch 
      || !reachedMaxPitch
      || fabs(inertial_sensor.roll(vex::deg) + 5) > 3);

  left_chassis.stop();
  right_chassis.stop();
}


void Motion::turnHeading(float target, int sides, float minSpeed, float maxSpeed, float timeout) {

  int timer = 0;

  turn.reset(Misc::angleError(target, Misc::getHeading(false)));
  (sides == 2) ? turn.setKp(Misc::angleError(target, Misc::getHeading(false))) : turn.setKp(Misc::angleError(target, Misc::getHeading(false)) / 2.5);

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    float output = Misc::clamp(turn.tick(Misc::angleError(target, Misc::getHeading(false))), -maxSpeed * 12000.0, maxSpeed * 12000.0);
    if (fabs(output) < minSpeed * 12000.0) output = copysign(minSpeed * 12000.0, output);

    if (sides == 0) Misc::runDrivetrain(output, 0, false);
    else if (sides == 1) Misc::runDrivetrain(0, -output, false);
    else Misc::runDrivetrain(output, -output, false);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (((minSpeed == 0 && Exit::velo(fabs(odometry.getAngularVelocity()), 3))
          || Exit::error(Misc::angleError(target, Misc::getHeading(false)), 4))
          && timer < timeout);

  headingTarget = target;
  Misc::runDrivetrain(0, 0, false);
  left_chassis.stop();
  right_chassis.stop();
}


void Motion::turnPoint(float x, float y, int sides, float minSpeed, float maxSpeed, float timeout) { 

  int timer = 0;

  Pose end(x, y, -1);
  Pose pose(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(false));

  turn.reset(pose.face(end, false));
  (sides == 2) ? turn.setKp(pose.face(end, false)) : turn.setKp(pose.face(end, false) / 2.5);

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();
    
    pose.set(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(false));

    float output = Misc::clamp(turn.tick(pose.face(end, false)), -maxSpeed * 12000.0, maxSpeed * 12000.0);
    if (fabs(output) < minSpeed * 12000.0) output = copysign(minSpeed * 12000.0, output);

    if (sides == 0) Misc::runDrivetrain(output, 0, false);
    else if (sides == 1) Misc::runDrivetrain(0, -output, false);
    else Misc::runDrivetrain(output, -output, false);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (((minSpeed == 0 && Exit::velo(fabs(odometry.getAngularVelocity()), 1))
          || Exit::error(pose.face(end, false), 1))
          && timer < timeout);

  headingTarget = Misc::getHeading(false);

  left_chassis.stop();
  right_chassis.stop();
}


void Motion::turnPointOneIter(float x, float y, int sides, float minSpeed, float maxSpeed) {

  Pose end(x, y, -1);
  Pose pose(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(false));

  float output = Misc::clamp(turn.tick(pose.face(end, false)), -maxSpeed * 12000.0, maxSpeed * 12000.0);
  if (fabs(output) < minSpeed * 12000.0) output = copysign(minSpeed * 12000.0, output);

  if (sides == 0) Misc::runDrivetrain(output, 0, false);
  else if (sides == 1) Misc::runDrivetrain(0, -output, false);
  else Misc::runDrivetrain(output, -output, false);

  headingTarget = Misc::getHeading(false);
}


void Motion::movePoint(float x, float y, float minSpeed, float maxSpeed, float timeout, bool settle, bool endIntake) {

  int timer = 0;

  float tolerance = (minSpeed == 0) ? 0.2 : 0.0;
  bool exitStatus = false, lastExitStatus = false, crossedOnce = false;

  Pose end(x, y, -1);
  Pose pose(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(maxSpeed < 0));

  lateral.reset(pose.distance(end));
  lateral.setKp(1);

  turn.reset(pose.face(end, false));
  turn.setKp(180);

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    lastExitStatus = Exit::hc(pose, end, Misc::getHeading(maxSpeed < 0), tolerance);

    pose.set(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(maxSpeed < 0));

    exitStatus = Exit::hc(pose, end, Misc::getHeading(maxSpeed < 0), tolerance);

    if (!exitStatus && lastExitStatus) crossedOnce = true;

    float distanceOutput = lateral.tick(pose.distance(end));
    if (settle && pose.distance(end) < 0.7) distanceOutput *= cos(Units::degToRad(pose.face(end, false)));
    distanceOutput = copysign(Misc::clamp(fabs(distanceOutput), fabs(minSpeed) * 12000.0, 12000.0), distanceOutput);
    
    float angleError = pose.face(end, false);
    float angularOutput = turn.tick(angleError);
    if (pose.distance(end) < 0.5) angularOutput = 0;
    angularOutput = Misc::clamp(angularOutput, -12000.0, 12000.0);

    float rescale = fabs(distanceOutput) + fabs(angularOutput) - 12000.0;
    if (rescale > 0) distanceOutput -= copysign(rescale, distanceOutput);

    std::pair<float, float> outputs = Misc::reduceRatio(fabs(maxSpeed) * 12000.0, distanceOutput + angularOutput, distanceOutput - angularOutput);
    Misc::runDrivetrain(outputs.first, outputs.second, maxSpeed < 0);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (((minSpeed == 0 && Exit::velo(odometry.getSpeed(), 0.5))
        || Exit::error(pose.distance(end), 0.6)
        || !crossedOnce)
        && (Exit::error(pose.distance(end), 0.5))
        && Exit::time(timer, timeout));

  headingTarget = Misc::getHeading(false);

  if (minSpeed == 0) {
    left_chassis.stop();
    right_chassis.stop();
  }
}


void Motion::movePose(float x, float y, float theta, float dLead, float gLead, float minSpeed, float maxSpeed, float timeout, float chasePower, bool settle, bool endIntake) {

  int timer = 0;
  float tolerance = (minSpeed == 0) ? 0.2 : 0.0;
  bool closeEnd = false, closeGhost = false, exitStatus = false, lastExitStatus = false, crossedOnce = false;

  if (maxSpeed < 0) theta += 180.0;

  Pose end(x, y, theta);
  Pose pose(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(maxSpeed < 0));
  Pose carrot(x - sin(Units::degToRad(theta)) * dLead, y - cos(Units::degToRad(theta)) * dLead, -1);
  Pose initialCarrot(carrot.x, carrot.y, -1);
  Pose target(0, 0, 0);

  float minCarrotDist = pose.distance(carrot);
  float initCarrotDist = pose.distance(carrot);

  lateral.reset(pose.distance(carrot));
  lateral.setKp(1);

  turn.reset(pose.face(carrot, false));
  turn.setKp(180);

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    lastExitStatus = Exit::hc(pose, end, theta, tolerance);

    pose.set(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(maxSpeed < 0));

    if (pose.distance(carrot) < minCarrotDist) minCarrotDist = pose.distance(carrot);
    carrot.set(x - (minCarrotDist / initCarrotDist) * sin(Units::degToRad(theta)) * dLead, y - (minCarrotDist / initCarrotDist) * cos(Units::degToRad(theta)) * dLead, -1);
    Pose ghost(initialCarrot.x + (carrot.x - initialCarrot.x) * (1 - gLead), initialCarrot.y + (carrot.y - initialCarrot.y) * (1 - gLead), -1);

    exitStatus = Exit::hc(pose, end, theta, tolerance);
    if (!exitStatus && lastExitStatus) crossedOnce = true;

    float angleError;
    if (pose.distance(carrot) < 0.7 || closeEnd) {
      closeEnd = true;
      if (pose.distance(end) < 0.7) angleError = pose.parallel(end);
      else angleError = pose.face(end, false);
      target.set(end.x, end.y, end.theta);
    }
    else if (pose.distance(ghost) < 2 * fabs(maxSpeed) || closeGhost) {
      closeGhost = true;
      angleError = pose.face(carrot, false);
      target.set(carrot.x, carrot.y, carrot.theta);
    }
    else {
      angleError = pose.face(ghost, false);
      target.set(ghost.x, ghost.y, ghost.theta);
    }

    float angularOutput = Misc::clamp(turn.tick(angleError), -12000.0, 12000.0);

    float distanceOutput = (closeEnd || pose.distance(end) > pose.distance(carrot)) ? lateral.tick(pose.distance(end)) : lateral.tick(pose.distance(carrot));
    
    if (settle && closeEnd && pose.distance(end) < 0.7) distanceOutput *= cos(Units::degToRad(pose.face(target, false)));
    distanceOutput = copysign(Misc::clamp(fabs(distanceOutput), fabs(minSpeed) * 12000.0, 12000.0), distanceOutput);

    float radius = Misc::getRadius(pose, target) / std::fmin(pose.distance(target), 1);
    float maxSlipSpeed = sqrt(chasePower * radius * 1000000);
    if (chasePower != -1 && (!closeEnd || pose.distance(end) >= 0.7)) distanceOutput = Misc::clamp(distanceOutput, -maxSlipSpeed, maxSlipSpeed);

    float rescale = fabs(distanceOutput) + fabs(angularOutput) - 12000.0;
    if (rescale > 0) distanceOutput -= copysign(rescale, distanceOutput);

    std::pair<float, float> outputs = Misc::reduceRatio(fabs(maxSpeed) * 12000.0, distanceOutput + angularOutput, distanceOutput - angularOutput);                                                                                              
    Misc::runDrivetrain(outputs.first, outputs.second, maxSpeed < 0);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (((minSpeed == 0 && Exit::velo(odometry.getSpeed(), 0.5))
        || !crossedOnce)
        && (Exit::error(pose.distance(end), 0.5))
        && Exit::time(timer, timeout));

  headingTarget = (maxSpeed < 0) ? theta - 180.0 : theta;
  
  if (minSpeed == 0) {
    left_chassis.stop();
    right_chassis.stop();
  }
}


void Motion::followPursuit(Path path, float timeout, float chasePower, bool reverse, bool endIntake) {
  
  int timer = 0, pathIndex = 0;
  Pose pose(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(reverse));

  do {
    
    uint64_t timestamp = vex::timer::systemHighResolution();

    pose.set(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(reverse));
    pathIndex = path.closestIndex(pose);

    float crosstrackError = pose.distance(path.at(pathIndex));
    float lookahead = Misc::clamp(odometry.getSpeed() * pursuitVeloConst + Misc::getRadius(pose, path.at(pathIndex + radiusLookahead)) * pursuitCurvConst - crosstrackError * pursuitCTEConst, minLookahead, maxLookahead);
    Pose lookaheadPoint = path.lookaheadPoint(pose, pathIndex, lookahead);

    float turnError = pose.face(lookaheadPoint, false);
    if (fabs(turnError) >= 90) {
      int dir = std::copysign(1.0, turnError);
      Misc::runDrivetrain(path.at(pathIndex).theta * dir, path.at(pathIndex).theta * -dir, reverse);
      continue;
    }

    float voltageScale = path.at(pathIndex).theta;
    if (chasePower != -1) {
      float radius = Misc::getRadius(pose, lookaheadPoint);
      voltageScale = std::fmin(sqrt(chasePower * radius / lookahead * 1000000), voltageScale);
    }

    float curvature = Misc::getCurvature(pose, lookaheadPoint);
    float leftRatio = 2 + curvature * odometry.getTrackWidth();
    float rightRatio = 2 - curvature * odometry.getTrackWidth();

    std::pair<float, float> outputs = Misc::scaleToRatio(voltageScale, leftRatio, rightRatio);
    Misc::runDrivetrain(outputs.first, outputs.second, reverse);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (((path.lastPoint().theta == 0 && Exit::velo(odometry.getSpeed(), 0.5)) 
        || pathIndex != path.lastIndex())
        && Exit::time(timer, timeout));

  headingTarget = Misc::getHeading(false);
  
  left_chassis.stop();
  right_chassis.stop();
}


void Motion::followStanley(Path path, float timeout, float k, float chasePower, bool reverse, bool endIntake) {
  
  int timer = 0, pathIndex = 0, furthestIndex = 0;
  Pose pose(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(reverse));

  do {
    
    uint64_t timestamp = vex::timer::systemHighResolution();

    pose.set(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(reverse));
    
    pathIndex = path.closestIndex(pose);
    if (pathIndex > furthestIndex) furthestIndex = pathIndex;
    else if (pathIndex < furthestIndex) pathIndex = furthestIndex;

    Pose curPoint = path.at(pathIndex);
    Pose nextPoint = path.at(pathIndex + 1);
    Pose prevPoint = path.at(pathIndex - 1);

    float pathHeading;
    if (pathIndex == path.lastIndex()) pathHeading = prevPoint.angle(curPoint);
    else pathHeading = curPoint.angle(nextPoint);

    float crosstrackError = std::copysign(pose.distance(curPoint), -Misc::getCurvature(curPoint, pose, pathHeading));
    //test Misc::getCurvature(pose, curPoint, pathHeading);

    float headingError = pose.parallel(pathHeading);
    float crosstrackScale = Units::radToDeg(atan(k * crosstrackError / odometry.getSpeed()));
    float steering = 5 * headingError + crosstrackScale;

    if (fabs(steering) >= 90) {
      int dir = std::copysign(1.0, steering);
      Misc::runDrivetrain(path.at(pathIndex).theta * dir, path.at(pathIndex).theta * -dir, reverse);
      continue;
    }

    float voltageScale = path.at(pathIndex).theta;
    if (chasePower != -1) {
      float radius = odometry.getTrackWidth() / tan(Units::degToRad(fabs(steering)));
      voltageScale = std::fmin(sqrt(chasePower * radius * 1000000), voltageScale);
    }

    float leftRatio = 2 + tan(Units::degToRad(steering)) / 2.0;
    float rightRatio = 2 - tan(Units::degToRad(steering)) / 2.0;

    std::pair<float, float> outputs = Misc::scaleToRatio(voltageScale, leftRatio, rightRatio);
    Misc::runDrivetrain(outputs.first, outputs.second, reverse);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (((path.lastPoint().theta == 0 && Exit::velo(odometry.getSpeed(), 0.5)) 
        || pathIndex != path.lastIndex())
        && Exit::time(timer, timeout));

  headingTarget = Misc::getHeading(false);
  
  left_chassis.stop();
  right_chassis.stop();
}


void Motion::followAPS(Path path, float timeout, float maxSpeed, float chasePower, bool reverse, bool endIntake) {

  int timer = 0, pathIndex = 0, furthestIndex = 0;
  Pose pose(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(reverse));

  turn.reset(0);
  turn.setKp(0);

  do {
    
    uint64_t timestamp = vex::timer::systemHighResolution();

    pose.set(odometry.getPosX(), odometry.getPosY(), Misc::getHeading(reverse));
    
    pathIndex = path.closestIndex(pose);
    if (pathIndex > furthestIndex) furthestIndex = pathIndex;
    else if (pathIndex < furthestIndex) pathIndex = furthestIndex;

    Pose curPoint = path.at(pathIndex);
    Pose nextPoint = path.at(pathIndex + 1);
    Pose prevPoint = path.at(pathIndex - 1);
    
    float pathHeading;
    if (pathIndex == 0) pathHeading = curPoint.angle(nextPoint);
    else if (pathIndex == path.lastIndex()) pathHeading = prevPoint.angle(curPoint);
    else pathHeading = 0.5 * (curPoint.angle(nextPoint) + prevPoint.angle(curPoint));
    
    float crosstrackError = pose.distance(curPoint);

    double angularTerm = pose.parallel(pathHeading);
    angularTerm = std::copysign(std::fmin(fabs(angularTerm / pow(25 * crosstrackError, 2)), fabs(angularTerm)), angularTerm);
    //printf("%f | %f\n", copysign(1, pose.face(curPoint, true)), copysign(1, -Misc::getCurvature(curPoint, pose, pathHeading)));
    float crosstrackTerm = (!path.nearEnd(pathIndex)) ? 10 * crosstrackError * pose.face(curPoint, true) : 0;
    float headingOutput = Misc::clamp(turn.tick(angularTerm + crosstrackTerm), -12000.0, 12000.0);
    //if (timer % 80 == 0) printf("%f | %f | %f | %f | %f\n", angularTerm, crosstrackTerm, crosstrackError, headingOutput, pathHeading);

    float lateralOutput = curPoint.theta;
    if (chasePower != -1) {
      Pose lookaheadPoint = path.lookaheadPoint(pose, pathIndex, 1.8);
      float radius = Misc::getRadius(pose, lookaheadPoint);
      lateralOutput = std::fmin(sqrt(chasePower * radius * 1000000), lateralOutput);
    }

    float rescale = fabs(lateralOutput) + fabs(headingOutput) - 1 * 12000.0;
    if (rescale > 0) lateralOutput -= copysign(rescale, lateralOutput);

    std::pair<float, float> outputs = Misc::reduceRatio(maxSpeed * 12000.0, lateralOutput + headingOutput, lateralOutput - headingOutput);                                                                                              
    Misc::runDrivetrain(outputs.first, outputs.second, reverse);

    timer += 10;
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (((path.lastPoint().theta == 0 && Exit::velo(odometry.getSpeed(), 0.5)) 
        || pathIndex != path.lastIndex())
        && Exit::time(timer, timeout));

  headingTarget = Misc::getHeading(false);
  
  left_chassis.stop();
  right_chassis.stop();
}