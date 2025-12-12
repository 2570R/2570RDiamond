#include "maths.h"


float rad2deg(float rad) {
    return rad * 180 / M_PI;
}

float deg2rad(float deg) {
    return deg * M_PI / 180;
}

float ema(float newValue, float oldValue, float alpha) {
    return (alpha * newValue) + ((1 - alpha) * oldValue);
}

float rot2inch(float rotations, float diameter) {
    return (diameter * M_PI * rotations) / 360;
}

float deg2inch(float deg, float diameter) {
    return (diameter * M_PI * deg) / 360;
}

float vel2rpm(float vel, float diameter) {
    return (vel * 60) / (diameter * M_PI);
}

float rpm2vel(float rpm, float diameter) {
    return (diameter * M_PI * rpm) / 60;
}

float dist(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

float sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}



float angleLerp(float a, float b, float t) {
    float diff = std::remainder(b - a, 2 * M_PI);
    return a + diff * t;
}

float angleErrorRaw(float target, float current) {
    float error = target - current;
    return std::atan2(std::sin(error), std::cos(error));  // returns error in [-π, π]
}

float slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

float getCurvature(Pose pose, Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    float side = sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

float curve(float input, float deadband, float curveGain, float minOutput) {
    if (fabs(input) <= deadband) return 0;
    else {
        // g is the output of g(x) as defined in the Desmos graph
        const float g = fabs(input) - deadband;
        // g127 is the output of g(127) as defined in the Desmos graph
        const float g127 = 127 - deadband;
        // i is the output of i(x) as defined in the Desmos graph
        const float i = pow(curveGain, g - 127) * g * sgn(input);
        // i127 is the output of i(127) as defined in the Desmos graph
        const float i127 = pow(curveGain, g127 - 127) * g127;
        return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * sgn(input);
    }
}

double project(Point point, Pose line) {
    double x = point.x - line.x;
    double y = point.y - line.y;
    double a = std::cos(line.theta);
    double b = std::sin(line.theta);
    return (x * a + y * b) / (a * a + b * b);
}