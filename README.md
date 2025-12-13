# 2570R - Code Documentation

## Table of Contents
1. [Asymptotic Gains](#asymptotic-gains)
2. [Feedforward and Friction Compensation](#feedforward-and-friction-compensation)
3. [Motion Profiling](#motion-profiling)
4. [Ramsete Controller](#ramsete-controller)
5. [Movement Functions](#movement-functions)
6. [File Structure](#file-structure)

---

## Asymptotic Gains

### Overview
Asymptotic Gains provide a dynamic way to adjust the proportional (Kp) value of PID controllers during movements. Instead of using a constant Kp value, the system calculates an adaptive gain that changes based on the current error (setpoint).

### How It Works
The `AsymptoticGains` class uses a mathematical formula to calculate the gain value:

```
gain = (f - i) * |setpoint|^p / (|setpoint|^p + k^p) + i
```

Where:
- **i** (initial): The minimum gain value when error is very large
- **f** (final): The maximum gain value when error approaches zero
- **k** (knee): The error value at which the gain transitions (inflection point)
- **p** (power): Controls the steepness of the transition curve
- **setpoint**: The current error value (distance from target)

### Visual Representation

**Gain vs Error Graph:**
```
Gain (Kp)
  ↑
  │
 f│                    ╭───────
  │                   ╱
  │                  ╱
  │                 ╱
  │                ╱
  │               ╱
  │              ╱
  │             ╱
 i│────────────╱
  │          ╱
  │         ╱
  └────────┼──────────────→ Error
            k (knee point)
```

**Example: angularKp = AsymptoticGains(480, 220, 28, 1.7)**
```
Kp Value
  ↑
480│●───────────────────
   │ ╲
   │  ╲
   │   ╲
   │    ╲
   │     ╲
   │      ╲
220│       ╲────────────●
   │        ╲
   │         ╲
   └──────────┼──────────→ Error (inches)
             28
      (knee point)
      
Interpretation:
- Large error (>28"): High Kp (480) for aggressive correction
- Small error (<28"): Low Kp (220) for fine-tuning
- Transition at 28" error
```

**Gain Adjustment During Movement:**
```
Movement Timeline:
Error:  ████████████░░░░░░░░░░  (decreasing)
        ↑                    ↑
      Start                Target

Kp:     ████░░░░░░░░░░░░░░░░░░  (adapting)
        ↑                    ↑
      High Kp            Low Kp
    (aggressive)      (precise)
```

### Implementation
In `robot-config.cpp`, three `AsymptoticGains` objects are defined:

```cpp
AsymptoticGains lateralKp = AsymptoticGains(15000, 15000, 1, 1);
AsymptoticGains angularKp = AsymptoticGains(480, 220, 28, 1.7);
AsymptoticGains correctKp = AsymptoticGains(200, 200, 1, 1);
```

These are then used with PID controllers:
- **lateralKp**: Used for lateral (sideways) movement control
- **angularKp**: Used for turning/rotational control - note the different initial (480) and final (220) values, allowing higher gain when far from target and lower gain when close
- **correctKp**: Used for heading correction during linear movement

### Active Adjustment During Movement
During a movement, the system continuously:
1. Calculates the current error (distance from target)
2. Calls `setGain(error)` to update the setpoint
3. Calls `getGain()` to retrieve the dynamically calculated Kp value
4. Uses this adaptive Kp in the PID controller's proportional term

This means:
- **Large errors**: Higher Kp value for aggressive correction
- **Small errors**: Lower Kp value for fine-tuning and preventing overshoot
- **Smooth transition**: The curve ensures gradual changes, preventing sudden jumps in control output

### Benefits
- **Adaptive response**: Automatically adjusts aggressiveness based on distance from target
- **Reduced overshoot**: Lower gain near target prevents oscillation
- **Faster convergence**: Higher gain when far away speeds up initial correction
- **Smoother control**: Gradual transitions prevent jerky movements

---

## Feedforward and Friction Compensation

### Overview
Feedforward control predicts the required motor output based on desired velocity, while friction compensation cancels out static friction that prevents smooth low-speed movement.

### Static Friction Feedforward (fStaticFF)
Static friction is the force that must be overcome to start moving. In joystick control, this manifests as a "dead zone" where small inputs don't produce movement.

The system implements friction compensation in `driveSettings.cpp`:

```cpp
float Chassis::getFStaticFF(float input) {
    if (input == 0) return 0;
    return copysign(chassisDriverSettings.fStaticFF, input);
}
```

This function:
- Returns 0 when input is 0 (no movement needed)
- Applies a constant voltage (`fStaticFF`) in the direction of movement to overcome static friction
- The sign matches the input direction

### Input to Output Conversion
The `inputToOutput` function combines feedforward and proportional control:

```cpp
float Chassis::inputToOutput(float input, float ff) {
    return input * (12000.0 - chassisDriverSettings.fStaticFF) / 127.0 + ff;
}
```

This formula:
- Scales the joystick input (0-127) to voltage (0-12000mV)
- Reserves `fStaticFF` voltage for friction compensation
- Adds the friction feedforward term to ensure movement starts immediately

### How It Works Together
1. **Joystick input** (0-127) is received
2. **Friction feedforward** is calculated based on input direction
3. **Proportional output** is calculated from the remaining voltage range
4. **Total output** = Proportional + Feedforward

**Visual Flow Diagram:**
```
Joystick Input (0-127)
        │
        ▼
┌───────────────────┐
│ Exponential Scale │
└────────┬──────────┘
         │
         ├─────────────────┐
         │                 │
         ▼                 ▼
┌─────────────────┐  ┌──────────────┐
│ Proportional    │  │ Friction FF  │
│ Output          │  │ (fStaticFF)  │
│ = input × scale │  │ = ±constant  │
└────────┬────────┘  └──────┬───────┘
         │                  │
         └────────┬──────────┘
                  ▼
         ┌─────────────────┐
         │ Total Output     │
         │ = Prop + FF      │
         └─────────────────┘
```

**Voltage Distribution:**
```
Total Available: 12000 mV
        │
        ├─── Friction FF: fStaticFF mV (reserved)
        │
        └─── Proportional: (12000 - fStaticFF) mV (scaled by input)
        
Example (fStaticFF = 500 mV, input = 64/127):
  Friction FF:    500 mV  ████
  Proportional:   5500 mV ████████████████████████████████████
  Total Output:   6000 mV
```

### Tuning the Friction Coefficient
The `fStaticFF` value in `ChassisDriverSettings` should be tuned to:
- **Too low**: Robot won't move with small joystick inputs (dead zone remains)
- **Too high**: Robot moves too aggressively, may overshoot at low speeds
- **Just right**: Smooth, responsive movement from zero input

Typical values range from 200-800mV depending on:
- Motor type and condition
- Gear ratio
- Wheel traction
- Surface friction

---

## Motion Profiling

### Overview
Motion profiling generates smooth, optimized paths using Bezier curves. The system creates waypoints with velocity and acceleration constraints to ensure smooth, efficient robot movement.

### Bezier Curves
A Bezier curve is defined by four control points (p0, p1, p2, p3) that create a smooth, continuous path:

```
    p1 ──────┐
             │
    p0 ──────┼─────── p3
             │
             └─────── p2
```

**Visual Representation:**
```
Control Points:
  p0: Start point (robot's current position)
  p1: First control point (influences start direction)
  p2: Second control point (influences end direction)
  p3: End point (target position)

Bezier Curve Path:
  p0 ────╮
         ╰───╮
             ╰─── p3
         p1      p2
```

### Waypoint Generation
The `BezierCurve::generateWaypoints()` function creates a series of waypoints along the curve:

**Process:**
1. **Calculate curve length** using numerical integration
2. **Determine waypoint spacing** (0.3 inches apart, minimum 2 waypoints)
3. **For each waypoint:**
   - Calculate position using Bezier formula: `P(t) = (1-t)³p0 + 3(1-t)²tp1 + 3(1-t)t²p2 + t³p3`
   - Calculate velocity (first derivative)
   - Calculate acceleration (second derivative)
   - Calculate curvature: `κ = (dx·ddy - dy·ddx) / (dx² + dy²)^1.5`
   - Calculate orientation (theta) from velocity direction

**Waypoint Structure:**
```cpp
struct Waypoint {
    float x, y;           // Position
    float dx, dy;         // Velocity components
    float ddx, ddy;       // Acceleration components
    float theta;          // Orientation (compass angle)
    float linvel;         // Linear velocity (in/s)
    float angvel;         // Angular velocity (rad/s)
    float t;              // Time stamp (ms)
};
```

### Velocity Constraints
The system applies multiple constraints to ensure safe, smooth motion:

**1. Maximum Linear Speed:**
```
max_linear_speed = (RPM / 60) * gear_ratio * wheel_diameter * π
Example: 450 RPM / 60 * (2/3) * 3.25" * π ≈ 50.9 in/s
```

**2. Curvature-Based Speed Limiting:**
```
curvature_limit = √(2 / |curvature|)
targetLinVel = min(targetLinVel, curvature_limit)
```

**Visual:**
```
High Curvature (Sharp Turn):
  ┌───┐
  │   │  ← Slow down here
  └───┘

Low Curvature (Gentle Curve):
  ╭───╮
  │   │  ← Can go faster
  ╰───╯
```

**3. Angular Velocity Calculation:**
```
angularVelocity = linearVelocity × curvature
```

### Waypoint Distribution
```
Path Visualization:
  p0 ──●───●───●───●───●───●───●── p3
       │   │   │   │   │   │   │
    Waypoint spacing: 0.3 inches
    Each waypoint has:
    - Position (x, y)
    - Velocity (dx, dy)
    - Acceleration (ddx, ddy)
    - Orientation (theta)
```

### Benefits
- **Smooth paths**: Bezier curves provide continuous, differentiable paths
- **Velocity profiling**: Automatic speed adjustment based on curvature
- **Acceleration limits**: Prevents sudden changes in velocity
- **Orientation tracking**: Each waypoint includes desired heading
- **Time-based**: Waypoints include timestamps for synchronization

---

## Ramsete Controller

### Overview
The Ramsete controller is a non-linear path-following algorithm that provides stable, convergent tracking of curved paths. It's used in `followPath()` to follow Bezier curve waypoints.

### How Ramsete Works
Ramsete converts path-following into a control problem by:
1. Transforming errors into the robot's local coordinate frame
2. Using a non-linear control law to drive errors to zero
3. Combining feedforward and feedback control

### Coordinate Frame Transformation
Errors are calculated in the robot's local frame, not the global frame:

```
Global Frame Error:
  Robot: (x_r, y_r, θ_r)
  Target: (x_t, y_t, θ_t)
  
  dx = x_t - x_r
  dy = y_t - y_r
  dθ = θ_t - θ_r

Robot Frame Error:
  error_x = cos(θ_r)·dx + sin(θ_r)·dy    (forward error)
  error_y = -sin(θ_r)·dx + cos(θ_r)·dy   (lateral error)
  error_θ = atan2(sin(dθ), cos(dθ))      (angular error)
```

**Visual Representation:**
```
Global Frame:
        y
        ↑
        │     Target (x_t, y_t)
        │        ●
        │         \
        │          \
        │           \
        │            ● Robot (x_r, y_r, θ_r)
        └────────────→ x

Robot Frame (rotated):
        ↑ (forward)
        │
        │  error_x (forward error)
        │    │
        │    │ error_y (lateral error)
        │    │  │
        └────┼──┼──→ (right)
             │  │
             ●  ●
           Robot Target
```

### Control Law
The Ramsete controller uses this non-linear control law:

```
k = 2·ζ·√(ω_t² + b·v_t²)

u1 = -k·error_x
u2 = -b·v_t·error_y - k·error_θ

v = v_t·cos(error_θ) - u1
ω = ω_t - u2
```

Where:
- **ζ (zeta)**: Damping ratio (0.7-1.0) - controls convergence speed
- **b**: Convergence parameter (1.0-3.0) - controls lateral error correction
- **v_t**: Target linear velocity (feedforward)
- **ω_t**: Target angular velocity (feedforward)
- **k**: Gain that adapts to target velocities

**Visual Control Flow:**
```
┌─────────────┐
│  Waypoint   │
│  (x_t,y_t,  │
│   v_t,ω_t)  │
└──────┬──────┘
       │
       ▼
┌─────────────────┐
│ Calculate Errors│
│ (robot frame)   │
└──────┬──────────┘
       │
       ▼
┌─────────────────┐      ┌──────────────┐
│ Ramsete Control │      │ Feedforward  │
│ Law (u1, u2)    │      │ (v_t, ω_t)   │
└──────┬──────────┘      └──────┬───────┘
       │                       │
       └───────────┬───────────┘
                   ▼
         ┌─────────────────┐
         │ Desired Velocities│
         │ v = v_t·cos(θ_e) │
         │   - u1           │
         │ ω = ω_t - u2     │
         └─────────┬─────────┘
                   ▼
         ┌─────────────────┐
         │ Wheel Velocities │
         │ v_L = v - ω·L/2 │
         │ v_R = v + ω·L/2 │
         └─────────────────┘
```

### Feedforward Gains
The system uses feedforward gains to convert waypoint velocities to motor voltages:

- **kV**: Linear velocity feedforward gain (volts per in/s)
  - Default: 2.4 V/(in/s)
  - Converts desired linear velocity to voltage
  
- **kW**: Angular velocity feedforward gain (volts per rad/s)
  - Default: 3.5 V/(rad/s)
  - Converts desired angular velocity to voltage

**Formula:**
```
v_t_voltage = v_t × kV
ω_t_voltage = ω_t × kW
```

### Lookahead and Waypoint Selection
The controller uses a lookahead algorithm to select the target waypoint:

```
1. Start from current waypoint index
2. Look ahead to find closest waypoint to robot
3. Select next waypoint (closest + 1) as target
4. This provides smooth following without oscillation
```

**Visual:**
```
Waypoints:  ●───●───●───●───●───●
            0   1   2   3   4   5
                    │
                 Robot here
                    │
                    ▼
            Closest: waypoint 2
            Target:  waypoint 3 (lookahead)
```

### Convergence Properties
Ramsete guarantees exponential convergence of errors to zero:

- **error_x**: Converges exponentially (forward error)
- **error_y**: Converges exponentially (lateral error)  
- **error_θ**: Converges exponentially (angular error)

**Tuning Parameters:**
- **Higher ζ**: Faster convergence, but may overshoot
- **Lower ζ**: Slower convergence, smoother
- **Higher b**: More aggressive lateral correction
- **Lower b**: Gentler lateral correction

### Implementation Details
The controller runs in a loop:
1. Get current robot pose (x, y, θ)
2. Find closest waypoint using lookahead
3. Get target waypoint (position, velocity, orientation)
4. Calculate errors in robot frame
5. Apply Ramsete control law
6. Convert to wheel velocities
7. Apply slew rate limiting
8. Drive motors
9. Repeat at 10ms intervals

**Error Convergence Visualization:**
```
Time →
error_x:  ████████░░░░░░░░░░░░  → 0
error_y:  ████████████░░░░░░░░  → 0
error_θ:  ██████░░░░░░░░░░░░░░  → 0
          ↑                    ↑
        Start              Converged
```

### Advantages Over PID
- **Non-linear paths**: Handles curved paths naturally
- **Coupled control**: Simultaneously corrects position and orientation
- **Guaranteed stability**: Mathematical proof of convergence
- **Feedforward**: Uses desired velocity for smoother motion
- **Robot-frame errors**: More intuitive error representation

---

## Movement Functions

All movement functions are located in `src/motor-control.cpp`. They follow a consistent pattern with similar parameters.

### Common Parameters
- **time_limit_msec**: Maximum time allowed for the movement (milliseconds)
- **exit**: If `true`, robot stops at end; if `false`, allows chaining movements
- **max_output**: Maximum voltage output to motors (typically 12V)

### Function Overview

#### `turnToAngle(turn_angle, time_limit_msec, exit, max_output)`
Turns the robot to a specified angle using PID control.

**Parameters:**
- `turn_angle`: Target angle in degrees
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end (true) or allow chaining (false)
- `max_output`: Maximum voltage (default: 12V)

**Features:**
- Uses `turnPID` with asymptotic gains
- Normalizes angle to ±180° range
- Visual feedback on Brain screen
- Supports chaining for continuous movement

---

#### `driveTo(distance_in, time_limit_msec, exit, max_output)`
Drives the robot a specified distance in a straight line using encoder feedback.

**Parameters:**
- `distance_in`: Distance to travel in inches (positive = forward, negative = backward)
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end or allow chaining
- `max_output`: Maximum voltage

**Features:**
- Uses encoder-based distance tracking
- Heading correction to maintain straight path
- Slew rate limiting for smooth acceleration/deceleration
- Supports chaining with adjustable slew rates

---

#### `driveToHeading(distance_in, targetHeading, time_limit_msec, exit, max_output)`
Drives a specified distance while maintaining a target heading.

**Parameters:**
- `distance_in`: Distance to travel in inches
- `targetHeading`: Target heading angle in degrees
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end or allow chaining
- `max_output`: Maximum voltage

**Features:**
- Combines distance PID with heading PID
- Maintains specific heading during movement
- Useful for precise positioning

---

#### `driveToDist(distance_mm, dir, time_limit_msec, exit, max_output)`
Drives to a specific distance using a distance sensor.

**Parameters:**
- `distance_mm`: Target distance in millimeters (from distance sensor)
- `dir`: Direction (1 = forward, -1 = backward)
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end or allow chaining
- `max_output`: Maximum voltage

**Features:**
- Uses front distance sensor for feedback
- Sensor-based positioning
- Heading correction included

---

#### `curveCircle(result_angle_deg, center_radius, time_limit_msec, exit, max_output)`
Drives the robot in a circular arc with a specified radius and ending angle.

**Parameters:**
- `result_angle_deg`: Target ending angle in degrees
- `center_radius`: Radius of the circle's center (positive = curve right, negative = curve left)
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end or allow chaining
- `max_output`: Maximum voltage

**Features:**
- Calculates inner and outer wheel arc lengths
- Maintains constant radius throughout curve
- Dynamic heading adjustment along the arc
- Supports both left and right curves

---

#### `swing(swing_angle, drive_direction, time_limit_msec, exit, max_output)`
Performs a swing turn, rotating around one wheel while driving forward/backward.

**Parameters:**
- `swing_angle`: Target angle to swing to in degrees
- `drive_direction`: Direction to drive (1 = forward, -1 = backward)
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end or allow chaining
- `max_output`: Maximum voltage

**Features:**
- Holds one side stationary, rotates the other
- Four swing modes: left/right, forward/backward combinations
- Useful for tight spaces or precise positioning

---

#### `turnToPoint(x, y, direction, time_limit_msec)`
Turns the robot to face a specific point in the field.

**Parameters:**
- `x, y`: Coordinates of the target point
- `direction`: Direction to face (1 = forward, -1 = backward)
- `time_limit_msec`: Maximum time allowed

**Features:**
- Calculates angle using `atan2`
- Continuously updates target as robot moves
- Visual feedback on Brain screen

---

#### `moveToPoint(x, y, dir, time_limit_msec, exit, max_output, overturn)`
Moves the robot to a specific point, adjusting heading as needed.

**Parameters:**
- `x, y`: Target coordinates
- `dir`: Direction to move (1 = forward, -1 = backward)
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end or allow chaining
- `max_output`: Maximum voltage
- `overturn`: Allow overturning for sharp turns (true/false)

**Features:**
- Combines distance and heading PID controllers
- Perpendicular line crossing detection for exit condition
- Overturn option for aggressive turns
- Heading correction only when far from target (>8 inches)

---

#### `moveToPointChain(x, y, dir, exit_dist, time_limit_msec, max_output, overturn)`
Chainable version of `moveToPoint` with distance-based exit.

**Parameters:**
- `x, y`: Target coordinates
- `dir`: Direction to move
- `exit_dist`: Exit when within this distance
- `time_limit_msec`: Maximum time allowed
- `max_output`: Maximum voltage
- `overturn`: Allow overturning

**Features:**
- Designed for chaining multiple movements
- Minimum speed enforcement
- Distance-based exit condition

---

#### `boomerang(x, y, dir, a, dlead, time_limit_msec, exit, max_output, overturn)`
Drives in a curved "boomerang" path to a target point with a specific ending angle.

**Parameters:**
- `x, y`: Target coordinates
- `dir`: Direction to move (1 = forward, -1 = backward)
- `a`: Final angle of robot at target (degrees)
- `dlead`: Distance to lead target by (0-0.6, higher = curvier path)
- `time_limit_msec`: Maximum time allowed
- `exit`: Stop at end or allow chaining
- `max_output`: Maximum voltage
- `overturn`: Allow overturning

**Features:**
- Uses "carrot point" leading for smooth curves
- Three-phase heading correction (carrot → target → final angle)
- Slip speed limiting for smooth curves
- Perpendicular line crossing detection

---

#### `followPath(p0, p1, p2, p3, exit, timeout_ms)`
Follows a Bezier curve path using the Ramsete controller.

**Parameters:**
- `p0, p1, p2, p3`: Control points for the Bezier curve
- `exit`: Stop at end or allow chaining
- `timeout_ms`: Maximum time allowed

**Features:**
- Generates waypoints from Bezier curve
- Uses Ramsete controller for non-linear path following
- Lookahead waypoint selection
- Robot-frame error calculation
- Feedforward velocity control (kV, kW gains)

**Path Visualization:**
```
Bezier Curve Path:
    p1 ────╮
           │
    p0 ────┼─────────── p3
           │
           └─────────── p2

Robot Following:
    p0 ────●───●───●───●───●───●─── p3
           │   │   │   │   │   │
         Robot waypoints (0.3" spacing)
```

---

## File Structure

```
2570RSpaceCity/
│
├── build/                          # Build output directory
│   ├── custom/
│   │   └── src/                    # Compiled object files
│   └── src/
│
├── custom/                         # User-customizable code
│   ├── include/
│   │   ├── autonomous.h           # Autonomous routine declarations
│   │   ├── intake.h                # Intake system declarations
│   │   ├── logger.h                # Logging system
│   │   ├── robot-config.h          # Robot configuration (motors, sensors, PID constants)
│   │   └── user.h                  # User control declarations
│   │
│   └── src/
│       ├── autonomous.cpp          # Autonomous routines implementation
│       ├── intake.cpp              # Intake system implementation
│       ├── logger.cpp              # Logging implementation
│       ├── robot-config.cpp        # Robot hardware configuration & PID tuning
│       └── user.cpp                # User control implementation
│
├── include/                        # Core library headers
│   ├── asymptoticGains.h          # Asymptotic gain class definition
│   ├── driveSettings.h            # Driver control settings (feedforward, friction)
│   ├── maths.h                    # Mathematical utility functions
│   ├── motor-control.h            # Movement function declarations
│   ├── mp.h                       # Motion planning (Bezier curves, waypoints)
│   ├── pid.h                      # PID controller class
│   ├── pose.h                     # Pose and Point classes
│   ├── utils.h                    # General utility functions
│   └── vex.h                      # VEX SDK header
│
├── src/                            # Core library implementation
│   ├── asymptoticGains.cpp       # Asymptotic gain implementation
│   ├── driveSettings.cpp          # Driver control implementation (friction compensation)
│   ├── maths.cpp                  # Math utilities implementation
│   ├── motor-control.cpp          # All movement functions implementation
│   ├── mp.cpp                     # Motion planning implementation
│   ├── pid.cpp                    # PID controller implementation
│   ├── utils.cpp                  # Utility functions implementation
│   └── main.cpp                   # Main program entry point
│
├── vex/                            # VEX build system files
│   ├── mkenv.mk                   # Build environment configuration
│   └── mkrules.mk                 # Build rules
│
├── makefile                        # Main build configuration
└── README.md                       # This file
```

### Key Files Explained

**`custom/src/robot-config.cpp`**
- Hardware declarations (motors, sensors)
- PID tuning constants
- Asymptotic gain configurations
- Robot geometry (track width, wheel diameter)

**`src/motor-control.cpp`**
- All movement function implementations
- Odometry tracking functions
- Chassis control primitives

**`src/driveSettings.cpp`**
- Joystick input processing
- Feedforward and friction compensation
- Exponential scaling
- Driver control modes (Tank, Arcade, Curvature)

**`include/asymptoticGains.h` & `src/asymptoticGains.cpp`**
- Asymptotic gain class definition and implementation
- Dynamic Kp calculation

**`include/pid.h` & `src/pid.cpp`**
- PID controller class
- Supports asymptotic gains for Kp
- Integral windup prevention
- Arrival detection

---

## Usage Examples

### Using Asymptotic Gains
```cpp
// In robot-config.cpp
AsymptoticGains turnKp = AsymptoticGains(480, 220, 28, 1.7);
PID turnPID = PID(turnKp, turn_ki, turn_kd);

// During movement
turnKp.setGain(current_error);  // Update based on current error
double kp_value = turnKp.getGain();  // Get adaptive Kp
```

### Chaining Movements
```cpp
// Chain movements without stopping
moveToPoint(10, 20, 1, 3000, false, 12, false);  // exit = false
moveToPoint(30, 40, 1, 3000, false, 12, false);  // Chains smoothly
moveToPoint(50, 60, 1, 3000, true, 12, false);   // Final movement stops
```

### Using followPath
```cpp
Point p0(0, 0);      // Start point
Point p1(10, 5);     // First control point
Point p2(20, 15);    // Second control point
Point p3(30, 20);    // End point

followPath(p0, p1, p2, p3, true, 5000);
```

---

## Tuning Guide

### Asymptotic Gains
- **i (initial)**: Start with 1.5x your desired max Kp
- **f (final)**: Use your desired steady-state Kp
- **k (knee)**: Set to typical error value where you want transition (e.g., 5-10 inches)
- **p (power)**: Higher = sharper transition (1.0-2.0 typical)

### Friction Compensation
- Start with 300mV and test
- Increase if robot doesn't respond to small inputs
- Decrease if robot moves too aggressively at low speeds
- Test on different surfaces

### Movement Functions
- **max_output**: Start with 12V, reduce if overshooting
- **time_limit_msec**: Set generously, function exits early on arrival
- **exit**: Use `false` for smooth chaining, `true` for stops

---

### Component Interaction Diagram
```
┌──────────────┐
│ Robot Config │───┐
│ - Hardware   │   │
│ - PID Gains  │   │
│ - Geometry   │   │
└──────────────┘   │
                   │
┌──────────────┐   │   ┌──────────────┐
│ Asymptotic   │───┼───│ PID          │
│ Gains        │   │   │ Controller   │
│ - Dynamic Kp │   │   │ - Uses AG    │
└──────────────┘   │   │ - Error calc │
                   │   └──────┬───────┘
┌──────────────┐   │          │
│ Motion       │───┼──────────┼───┐
│ Planning     │   │          │   │
│ - Bezier     │   │          │   │
│ - Waypoints  │   │          │   │
└──────────────┘   │          │   │
                   │          │   │
┌──────────────┐   │          │   │
│ Ramsete      │───┼──────────┼───┼───┐
│ Controller   │   │          │   │   │
│ - Path follow│   │          │   │   │
└──────────────┘   │          │   │   │
                   │          │   │   │
                   ▼          ▼   ▼   ▼
            ┌─────────────────────────────┐
            │   Motor Control System      │
            │   - driveChassis()          │
            │   - Voltage Output          │
            └─────────────────────────────┘
```

## Notes

- Hiii

