#include <string>
#include <cmath>
#include "mp.h"
#include "maths.h"
#include "pose.h"

// --- Global Variables (snake_case) ---
extern bool is_turning;

extern double x_pos, y_pos;
extern double correct_angle;

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading(bool normalize = false);
double normalizeTarget(double angle);
void driveToHeading(double distance_in, double targetHeading, double time_limit_msec, bool exit = true, double max_output = 12);
void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);
void resetAngle(double angle);
void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
void resetOdom(double _x, double _y);
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void moveToPointChain(double x, double y, int dir, double exit_dist, double time_limit_msec, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void driveToDist(double distance_mm, int dir, double time_limit_msec, bool exit = true, double max_output = 12);
void followPath(Point p0, Point p1, Point p2, Point p3, int timeout_ms);