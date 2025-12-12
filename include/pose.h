#pragma once

class Pose {
    public:
        float x;
        float y;
        float theta;
        Pose();
        Pose(float x, float y, float theta = 0);
        Pose operator+(const Pose& other) const;
        Pose operator-(const Pose& other) const;
        float operator*(const Pose& other) const;
        Pose operator*(const float& other) const;
        Pose operator/(const float& other) const;
        Pose lerp(Pose other, float t) const;
        float distance(Pose other) const;
        float angle(Pose other) const;
        Pose rotate(float angle) const;
};

class Point {
    public:
        double x;
        double y;
        Point();
        Point(double x, double y);
        Point operator+(const Point& other) const;
        Point operator-(const Point& other) const;
        double operator*(const Point& other) const;
        Point operator*(const double& other) const;
        Point operator/(const double& other) const;
        double distance(Point other) const;
};