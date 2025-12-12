#pragma once

#include "pose.h"
#include <vector>

struct Waypoint {
    float x;
    float y;
    float dx;
    float dy;
    float ddx;
    float ddy;
    float theta;
    float linvel;
    float angvel;
    float t;
};

Point getPoint(Point p0, Point p1, Point p2, Point p3, float t);

Point getDerivative(Point p0, Point p1, Point p2, Point p3, float t);

Point getSecondDerivative(Point p0, Point p1, Point p2, Point p3, float t);

float getBezierCurvature(Point p0, Point p1, Point p2, Point p3, float t);

class BezierCurve {

    public:
        BezierCurve() = default;
        BezierCurve(Point path[4]) : p0(path[0]), p1(path[1]), p2(path[2]), p3(path[3]) {};
        BezierCurve(Point p0, Point p1, Point p2, Point p3) : p0(p0), p1(p1), p2(p2), p3(p3) {};
        BezierCurve(const BezierCurve& other) : waypoints(other.waypoints), p0(other.p0), p1(other.p1), p2(other.p2), p3(other.p3) {}

        std::vector<Waypoint> waypoints;
        Point p0, p1, p2, p3;
        void generateWaypoints();
        std::vector<Waypoint> getWaypoints() { return waypoints; }
};