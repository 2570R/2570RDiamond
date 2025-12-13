#include "motionControl/Odom.h"

TwoWheelInertialOdometry::TwoWheelInertialOdometry(OdometrySensors& sensors, OdometryOffsets& offsets, float unit)
    //constructor for odometry object with two tracking wheels and an inertial sensor

    : sensors(sensors)
    , offsets(offsets)
    , unit(unit)
{}


void TwoWheelInertialOdometry::updateVelocities() {
    //updates data storage vectors for robot state calculations

    positions.push_back(Pose(pose.x, pose.y));
    if (positions.size() > 4) positions.erase(positions.begin());
    angles.push_back(sensors.inertial->getRotation());
    if (angles.size() > 4) angles.erase(angles.begin());
}


float TwoWheelInertialOdometry::getVelocityX() {
    //returns approximated velocity on the horizontal axis in units per second

    return (!positions.empty()) ? (positions.back().x - positions.front().x) * (100.0 / positions.size()) : 0;
}


float TwoWheelInertialOdometry::getVelocityY() {
    //returns approximated velocity on the vertical axis in units per second

    return (!positions.empty()) ? (positions.back().y - positions.front().y) * (100.0 / positions.size()) : 0;
}


void TwoWheelInertialOdometry::updatePosition() {
    //updates the position estimate using new telemetric data

    float perpendicular = Units::tickToUnit(sensors.perpendicular->sensor.position(vex::deg), sensors.perpendicular->diameter, unit);
    float parallel = Units::tickToUnit(sensors.parallel->sensor.position(vex::deg), sensors.parallel->diameter, unit);
    float rotation = sensors.inertial->getRotation(true);

    float deltaPerpendicular = perpendicular - lastPerpendicular;
    float deltaParallel = parallel - lastParallel;
    float deltaRotation = rotation - lastRotation;

    float relOffsetX = (deltaRotation == 0) ? deltaPerpendicular : 2.0 * sin(deltaRotation / 2.0) * (deltaPerpendicular / deltaRotation + offsets.perpendicularOffset);
    float relOffsetY = (deltaRotation == 0) ? deltaParallel : 2.0 * sin(deltaRotation / 2.0) * (deltaParallel / deltaRotation + offsets.parallelOffset);
  
    float deltaX = sqrt(pow(relOffsetX, 2.0) + pow(relOffsetY, 2.0)) * cos(atan2(relOffsetY, relOffsetX) - (lastRotation + deltaRotation / 2.0));
    float deltaY = sqrt(pow(relOffsetX, 2.0) + pow(relOffsetY, 2.0)) * sin(atan2(relOffsetY, relOffsetX) - (lastRotation + deltaRotation / 2.0));

    pose.set(pose.x + deltaX, pose.y + deltaY, sensors.inertial->getHeading());

    lastPerpendicular = perpendicular;
    lastParallel = parallel;
    lastRotation = rotation;

    updateVelocities();
}


void TwoWheelInertialOdometry::setPosition(float x, float y) {
    //sets the x and y coordinates of the odometry to a user-specified number

    pose.set(x, y, pose.theta);
}


float TwoWheelInertialOdometry::getUnit() {
    //returns the unit of the odometry object

    return unit;
}


Pose TwoWheelInertialOdometry::getPose() {
    //returns the location and heading of the robot
    
    return pose;
}


float TwoWheelInertialOdometry::getSpeed() {
    //returns the speed that the robot is traveling at using odometry data in units per second

    return sqrt(pow(getVelocityX(), 2) + pow(getVelocityY(), 2));
}


float TwoWheelInertialOdometry::getAngularVelocity() {
    //returns the angular velocity that the robot is turning at in degrees per second

    return (!angles.empty()) ? (angles.back() - angles.front()) * (100.0 / angles.size()) : 0;
}


float TwoWheelInertialOdometry::getTranslationDirection() {
    //returns the heading of the robot's motion in degrees

    return (getSpeed() < 0.1) ? 0 : Units::radToDeg(atan2(getVelocityX(), getVelocityY()));
}