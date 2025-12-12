#pragma once

#include <vector>
#include <cmath>
#include <initializer_list>
#include "pose.h"

#define DT_WHEEL_DIAMETER 3.25

template <typename T>
int sgn(T x) {
    return (T(0) < x) - (x < T(0));
}

template <typename T>
float avg(const std::vector<T>& values) {
    float sum = 0;
    for (const auto& value : values) {
        sum += value;
    }
    return sum / values.size();
}

template <typename T>
float avg(const std::initializer_list<T>& values) {
    float sum = 0;
    for (const auto& value : values) {
        sum += value;
    }
    return sum / values.size();
}

template <typename T>
float mod(T a, T b) {
    return a - b * floor(a / b);
}

/*** @brief only numbers!!!!!!!!!! ***/
template <typename T>
float avg(const std::vector<T>& values);

template <typename T>
float avg(const std::initializer_list<T>& values);

float rad2deg(float rad);

float deg2rad(float deg);

// alpha is weight of new value
float ema(float newValue, float oldValue, float alpha);

Pose smoothPose(const Pose& newPose, const Pose& oldPose, float alpha);

// uses drivetrain wheel diameter by default
float rot2inch(float rotations, float diameter = DT_WHEEL_DIAMETER);

// uses drivetrain wheel diameter by default
float deg2inch(float deg, float diameter = DT_WHEEL_DIAMETER);

// vel is in inches per second
float vel2rpm(float vel, float diameter = DT_WHEEL_DIAMETER);

// vel is in inches per second
float rpm2vel(float rpm, float diameter = DT_WHEEL_DIAMETER);

float dist(float x1, float y1, float x2, float y2);

float sanitizeAngle(float angle, bool radians = false);


float angleErrorRaw(float target, float current);

float angleLerp(float a, float b, float t);

float slew(float target, float current, float maxChange);

float getCurvature(Pose pose, Pose other);

float curve(float input, float deadband, float curveGain, float minOutput = 0);

double project(Point point, Pose line);