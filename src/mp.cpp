#include "mp.h"
#include "maths.h"

#include <iostream>

double bezierLength(Point p0, Point p1, Point p2, Point p3, int steps = 100) {
    double length = 0.0;
    Point prev = getPoint(p0, p1, p2, p3, 0.0);
    for (int i = 1; i <= steps; ++i) {
        double t = (double)i / steps;
        Point curr = getPoint(p0, p1, p2, p3, t);
        length += dist(prev.x, prev.y, curr.x, curr.y);
        prev = curr;
    }
    return length;
}

Point getPoint(Point p0, Point p1, Point p2, Point p3, float t) {
    float u = 1 - t;
    float b0 = u * u * u;
    float b1 = 3 * u * u * t;
    float b2 = 3 * u * t * t;
    float b3 = t * t * t;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

Point getDerivative(Point p0, Point p1, Point p2, Point p3, float t) {
    float u = 1 - t;
    float b0 = 3 * u * u;
    float b1 = 6 * u * t;
    float b2 = 3 * t * t;
    return p0 * b0 + p1 * b1 + p2 * b2;
}

Point getSecondDerivative(Point p0, Point p1, Point p2, Point p3, float t) {
    float u = 1 - t;
    float b0 = 6 * u;
    float b1 = 6 * t;
    return p0 * b0 + p1 * b1;
}

float getBezierCurvature(Point p0, Point p1, Point p2, Point p3, float t) {
    Point d = getDerivative(p0, p1, p2, p3, t);
    Point dd = getSecondDerivative(p0, p1, p2, p3, t);
    return (d.x * dd.y - d.y * dd.x) / pow(d.x * d.x + d.y * d.y, 1.5);
}

// Path& Path::operator=(const Path& other) {
//     if (this != &other) {
//         waypoints = other.waypoints;
//     }
//     return *this;
// }

void BezierCurve::generateWaypoints() {

    // Number of waypoints to generate
    double currentTime = 0;
    float prevX = p0.x;
    float prevY = p0.y;

    double segmentLength = bezierLength(p0, p1, p2, p3);
    int numWaypoints = std::max(2, static_cast<int>(segmentLength / 0.3)); // minimum 2 waypoints

    waypoints.clear();
    waypoints.reserve(numWaypoints);

    for (int i = 0; i <= numWaypoints; ++i) {
        float t = (float)i / numWaypoints;
        Point currentPoint = getPoint(p0, p1, p2, p3, t);

        Point velocity = getDerivative(p0, p1, p2, p3, t);
        Point acceleration = getSecondDerivative(p0, p1, p2, p3, t);

        float rawVelocityMag = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

        // Estimate a "desired" velocity: interpolate between start and end (you could use a real table here)
        float targetLinVel = rawVelocityMag; // simple for now, could replace with interpolated value

        // Scale raw velocity to inches/sec based on robot limits
        double max_linear_speed = 450.0 / 60.0 * (2/3) * 3.25 * M_PI; // in/s
        targetLinVel = std::fmin(targetLinVel, max_linear_speed);

        // Calculate curvature and limit velocity if needed
        float curvature = getBezierCurvature(p0, p1, p2, p3, t);
        if (std::abs(curvature) > 1e-5) {
            double curvature_limit = std::sqrt(2.0 / std::abs(curvature)); // or use your robot's max lateral accel
            targetLinVel = std::fmin(targetLinVel, curvature_limit);
        }

        // Calculate angular velocity
        float angularVelocity = targetLinVel * curvature;

        // Calculate theta (orientation)
        float theta = std::atan2(velocity.y, velocity.x);
        theta = fmod(M_PI / 2 - theta, 2 * M_PI); // converts to compass angle

        // Protect against zero velocity when computing deltaTime
        double safeLinVel = std::max(targetLinVel, 1e-5f);
        double deltaTime = dist(prevX, prevY, currentPoint.x, currentPoint.y) / safeLinVel;
        currentTime += deltaTime * 1000.0; // ms

        // Fill Waypoint
        Waypoint wp;
        wp.x = currentPoint.x;
        wp.y = currentPoint.y;
        wp.dx = velocity.x;
        wp.dy = velocity.y;
        wp.ddx = acceleration.x;
        wp.ddy = acceleration.y;
        wp.theta = theta;
        wp.linvel = targetLinVel;
        wp.angvel = angularVelocity;
        wp.t = currentTime;

        waypoints.push_back(wp);

        prevX = currentPoint.x;
        prevY = currentPoint.y;
    }
}