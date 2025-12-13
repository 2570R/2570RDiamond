# 2570R - Code Documentation

## Table of Contents
1. [Asymptotic Gains](#asymptotic-gains)
2. [Feedforward and Friction Compensation](#feedforward-and-friction-compensation)
3. [Motion Profiling](#motion-profiling)
4. [Ramsete Controller](#ramsete-controller)
5. [Stanley Controller](#stanley-controller)
6. [Pure Pursuit](#pure-pursuit)
7. [APS Controller](#aps-controller)
8. [Odometry Reset with Distance Sensors](#odometry-reset-with-distance-sensors)
9. [Movement Functions](#movement-functions)
10. [File Structure](#file-structure)

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

## Stanley Controller

### Overview
The Stanley controller is a path-following algorithm originally developed for autonomous vehicles. It combines heading error correction with crosstrack error compensation to provide stable path following. The controller is implemented in `followStanley()` and is particularly effective for high-speed path following.

### How Stanley Works
The Stanley controller calculates a steering angle based on two components:
1. **Heading Error**: Difference between robot heading and path heading
2. **Crosstrack Error**: Perpendicular distance from the robot to the path

### Control Law
The steering angle is calculated as:

```
steering = K_h × headingError + crosstrackScale

where:
  headingError = robot_heading - path_heading
  crosstrackScale = atan(k × crosstrackError / velocity)
```

Where:
- **K_h**: Heading error gain (default: 5)
- **k**: Crosstrack error gain (tunable parameter)
- **crosstrackError**: Signed perpendicular distance to path
- **velocity**: Current robot speed

**Visual Representation:**
```
Path:
  ────────────────
         │
         │ crosstrackError
         │
         ● Robot
         │
    headingError
         │
    Path Heading ───→
```

### Path Following Process
1. **Find closest path point**: Uses `path.closestIndex()` to find nearest waypoint
2. **Calculate path heading**: Determines direction of path at current point
3. **Compute errors**:
   - Crosstrack error: Signed distance to path (positive = left of path)
   - Heading error: Difference between robot and path headings
4. **Calculate steering angle**: Combines both error terms
5. **Convert to wheel velocities**: Uses Ackermann steering geometry

### Crosstrack Error Calculation
The crosstrack error is signed based on which side of the path the robot is on:

```cpp
crosstrackError = sign × distance(curPoint, robot)
sign = -curvature(curPoint, robot, pathHeading)
```

This ensures:
- **Positive error**: Robot is to the left of path → steer right
- **Negative error**: Robot is to the right of path → steer left

### Steering Angle to Wheel Velocities
The steering angle is converted to differential drive using Ackermann geometry:

```
leftRatio = 2 + tan(steering) / 2.0
rightRatio = 2 - tan(steering) / 2.0
```

**Visual:**
```
Steering Angle:
  Positive (left turn):
    ┌───┐
    │ R │  ← Robot
    └───┘
     ╱
    ╱  steering angle
   ╱
  
  Wheel Velocities:
    Left wheel:  faster
    Right wheel: slower
```

### Dynamic Lookahead (Velocity-Based)
The crosstrack correction term includes velocity in the denominator:

```
crosstrackScale = atan(k × crosstrackError / velocity)
```

This means:
- **High speed**: Smaller correction (less aggressive)
- **Low speed**: Larger correction (more aggressive)
- **Prevents overshoot**: At high speeds, corrections are gentler

### Path Index Management
The controller uses a "furthest index" system to prevent backtracking:

```cpp
if (pathIndex > furthestIndex) furthestIndex = pathIndex;
else if (pathIndex < furthestIndex) pathIndex = furthestIndex;
```

This ensures the robot always progresses forward along the path, even if it temporarily moves backward due to error.

### Chase Power (Speed Limiting)
Similar to Pure Pursuit, Stanley supports speed limiting based on turning radius:

```cpp
if (chasePower != -1) {
  radius = trackWidth / tan(steering_angle);
  voltageScale = min(sqrt(chasePower × radius × 1000000), voltageScale);
}
```

This prevents the robot from exceeding safe speeds during sharp turns.

### Implementation Details
The controller runs in a loop:
1. Get current robot pose
2. Find closest path point (with furthest index tracking)
3. Calculate path heading from adjacent points
4. Compute crosstrack and heading errors
5. Calculate steering angle
6. Convert to wheel velocities using Ackermann geometry
7. Apply chase power speed limiting
8. Drive motors
9. Repeat at 10ms intervals

### Tuning Parameters
- **k (crosstrack gain)**: Controls aggressiveness of crosstrack correction
  - Higher k: More aggressive correction
  - Lower k: Gentler correction
  - Typical range: 0.5 - 5.0
- **K_h (heading gain)**: Multiplier for heading error (default: 5)
  - Higher: More responsive to heading errors
  - Lower: Smoother but less responsive

### Advantages
- **Velocity-dependent**: Automatically adjusts correction based on speed
- **Simple tuning**: Only one main parameter (k) to tune
- **Smooth following**: Natural steering behavior
- **High-speed capable**: Works well at various speeds

### When to Use
- High-speed path following
- When you need simple, tunable path following
- When velocity-dependent correction is desired
- For paths with varying curvature

---

## Pure Pursuit

### Overview
Pure Pursuit is a geometric path-following algorithm that uses a "lookahead point" ahead of the robot on the path. The robot steers toward this lookahead point, creating smooth, natural following behavior. The implementation uses dynamic lookahead distance that adapts based on speed, curvature, and crosstrack error.

### How Pure Pursuit Works
Pure Pursuit works by:
1. Finding a lookahead point ahead on the path
2. Calculating the curvature needed to reach that point
3. Converting curvature to wheel velocities
4. Driving toward the lookahead point

### Lookahead Distance
The lookahead distance is dynamically calculated based on multiple factors:

```
lookahead = clamp(
  velocity × pursuitVeloConst + 
  radius × pursuitCurvConst - 
  crosstrackError × pursuitCTEConst,
  minLookahead,
  maxLookahead
)
```

Where:
- **pursuitVeloConst**: Velocity contribution to lookahead
- **pursuitCurvConst**: Curvature contribution to lookahead
- **pursuitCTEConst**: Crosstrack error contribution (reduces lookahead when off-path)
- **minLookahead**: Minimum lookahead distance
- **maxLookahead**: Maximum lookahead distance

**Visual Representation:**
```
Path:
  ●───●───●───●───●───●
       │       │
       │       └─── Lookahead Point
       │           (lookahead distance)
       │
       └─── Robot
            (crosstrackError)
```

### Dynamic Lookahead Components

**1. Velocity Component:**
```
velocity × pursuitVeloConst
```
- Higher speed → larger lookahead
- Allows robot to "see further ahead" at high speeds
- Prevents cutting corners

**2. Curvature Component:**
```
radius × pursuitCurvConst
```
- Uses radius from robot to a point further ahead (`pathIndex + radiusLookahead`)
- Sharp turns → larger lookahead (smoother following)
- Straight paths → smaller lookahead (tighter following)

**3. Crosstrack Error Component:**
```
-crosstrackError × pursuitCTEConst
```
- When off-path, reduces lookahead
- Forces robot to correct more aggressively
- Helps recover from large errors

### Curvature Calculation
Once the lookahead point is found, the curvature to reach it is calculated:

```
curvature = getCurvature(robot, lookaheadPoint)
```

The curvature is then converted to wheel velocity ratios:

```
leftRatio = 2 + curvature × trackWidth
rightRatio = 2 - curvature × trackWidth
```

**Visual:**
```
Positive Curvature (left turn):
  ┌───┐
  │ R │  ← Robot
  └───┘
   ╱
  ╱  curvature
 ╱
● Lookahead Point

Wheel Velocities:
  Left:  slower (leftRatio < 2)
  Right: faster (rightRatio > 2)
```

### Lookahead Point Selection
The algorithm finds the lookahead point by:
1. Starting from the closest path point
2. Searching forward along the path
3. Finding the point exactly `lookahead` distance away
4. Using interpolation if between waypoints

### Chase Power (Speed Limiting)
Pure Pursuit supports speed limiting based on turning radius:

```cpp
if (chasePower != -1) {
  radius = getRadius(robot, lookaheadPoint);
  voltageScale = min(sqrt(chasePower × radius / lookahead × 1000000), voltageScale);
}
```

This prevents the robot from exceeding safe speeds during sharp turns.

### Configuration
The lookahead parameters are set using `setPursuitSettings()`:

```cpp
setPursuitSettings(
  minLookahead,      // Minimum lookahead distance
  maxLookahead,      // Maximum lookahead distance
  pursuitVeloConst,  // Velocity contribution
  pursuitCurvConst,  // Curvature contribution
  pursuitCTEConst,   // Crosstrack error contribution
  radiusLookahead    // How far ahead to look for curvature
);
```

### Implementation Flow
1. Get current robot pose
2. Find closest path point
3. Calculate dynamic lookahead distance
4. Find lookahead point on path
5. Calculate curvature to reach lookahead point
6. Convert curvature to wheel velocity ratios
7. Apply chase power speed limiting
8. Drive motors
9. Repeat at 10ms intervals

### Tuning Guide
- **minLookahead**: Start with 6-12 inches
  - Too small: Oscillatory behavior
  - Too large: Slow response, cuts corners
- **maxLookahead**: Start with 18-24 inches
  - Limits maximum lookahead for safety
- **pursuitVeloConst**: Start with 0.1-0.3
  - Controls how lookahead scales with speed
- **pursuitCurvConst**: Start with 0.5-1.5
  - Controls how lookahead scales with curvature
- **pursuitCTEConst**: Start with 0.1-0.5
  - Controls how crosstrack error reduces lookahead

### Advantages
- **Smooth following**: Natural steering behavior
- **Adaptive**: Lookahead adjusts to conditions
- **Geometric**: Intuitive to understand and tune
- **Robust**: Works well on various path types

### When to Use
- Smooth, curved path following
- When you want adaptive lookahead behavior
- For paths with varying curvature
- When you need predictable, geometric control

---

## APS Controller

### Overview
APS (Adaptive Path-following System) is a hybrid path-following controller that combines angular error correction with crosstrack error compensation. It uses a PID controller for heading correction and adaptively scales corrections based on distance from the path. APS is designed to provide smooth, stable path following with good convergence properties.

### How APS Works
APS uses two main correction terms:
1. **Angular Term**: Corrects heading error relative to path heading
2. **Crosstrack Term**: Corrects lateral position error

These terms are combined and fed into a PID controller for smooth heading correction.

### Path Heading Calculation
APS calculates the path heading at the current point using adjacent waypoints:

```cpp
if (pathIndex == 0) 
  pathHeading = curPoint.angle(nextPoint);
else if (pathIndex == lastIndex) 
  pathHeading = prevPoint.angle(curPoint);
else 
  pathHeading = 0.5 × (curPoint.angle(nextPoint) + prevPoint.angle(curPoint));
```

This provides:
- **Start of path**: Uses forward direction
- **End of path**: Uses backward direction
- **Middle of path**: Averages forward and backward directions for smoothness

### Angular Term
The angular term corrects the robot's heading to match the path heading:

```cpp
angularTerm = pose.parallel(pathHeading)
angularTerm = clamp(angularTerm / (25 × crosstrackError)², angularTerm)
```

**Adaptive Scaling:**
- When **far from path** (large crosstrack error): Angular term is reduced
- When **close to path** (small crosstrack error): Full angular correction
- This prevents over-correction when far off-path

**Visual:**
```
Far from Path:
  ┌───┐
  │ R │  ← Robot (far off)
  └───┘
       │
       │ large crosstrackError
       │
  ─────┼───── Path
       │
  Reduced angular correction

Close to Path:
  ┌───┐
  │ R │  ← Robot (close)
  └───┘
    │
    │ small crosstrackError
    │
  ──┼─── Path
    │
  Full angular correction
```

### Crosstrack Term
The crosstrack term corrects lateral position error:

```cpp
crosstrackTerm = 10 × crosstrackError × sign(face(curPoint))
```

Where:
- **crosstrackError**: Distance from robot to closest path point
- **sign(face(curPoint))**: Direction to turn toward path
- **10**: Crosstrack gain (fixed)

**Disabled near end:**
- When `path.nearEnd(pathIndex)` is true, crosstrack term is set to 0
- Prevents overshoot at path endpoint

### Combined Control
The two terms are combined and fed into a PID controller:

```cpp
headingOutput = PID.tick(angularTerm + crosstrackTerm)
```

The PID controller provides:
- Smooth transitions
- Damping to prevent oscillation
- Integral action for steady-state accuracy

### Path Index Management
Similar to Stanley, APS uses furthest index tracking:

```cpp
if (pathIndex > furthestIndex) furthestIndex = pathIndex;
else if (pathIndex < furthestIndex) pathIndex = furthestIndex;
```

This ensures forward progress along the path.

### Speed Control
APS uses the path's velocity profile directly:

```cpp
lateralOutput = curPoint.theta  // Path velocity
```

With optional chase power limiting:

```cpp
if (chasePower != -1) {
  Pose lookaheadPoint = path.lookaheadPoint(pose, pathIndex, 1.8);
  float radius = getRadius(pose, lookaheadPoint);
  lateralOutput = min(sqrt(chasePower × radius × 1000000), lateralOutput);
}
```

### Output Scaling
The lateral and heading outputs are combined with rescaling to prevent saturation:

```cpp
rescale = |lateralOutput| + |headingOutput| - 12000.0
if (rescale > 0) lateralOutput -= sign(rescale) × rescale
```

This ensures the combined output never exceeds motor limits.

### Implementation Flow
1. Get current robot pose
2. Find closest path point (with furthest index tracking)
3. Calculate path heading from adjacent points
4. Compute crosstrack error
5. Calculate angular term (with adaptive scaling)
6. Calculate crosstrack term (disabled near end)
7. Combine terms and feed to PID controller
8. Get path velocity and apply chase power limiting
9. Rescale outputs to prevent saturation
10. Convert to wheel velocities
11. Drive motors
12. Repeat at 10ms intervals

### Tuning Parameters
- **maxSpeed**: Maximum speed multiplier (default: 1.0)
  - Controls overall path following speed
- **chasePower**: Speed limiting constant (optional)
  - Limits speed based on turning radius
  - Set to -1 to disable
- **PID gains**: Tuned via the `turn` PID controller
  - Note: PID is reset with Kp=0 at start, so gains should be set elsewhere

### Advantages
- **Adaptive correction**: Scales angular correction based on distance
- **Smooth control**: PID provides damping
- **Dual correction**: Both heading and position errors addressed
- **Endpoint handling**: Disables crosstrack term near end to prevent overshoot

### When to Use
- When you need smooth, stable path following
- For paths where adaptive correction is beneficial
- When you want PID-based heading control
- For paths with varying error conditions

---

## Odometry Reset with Distance Sensors

### Overview
The robot uses three distance sensors (front, left, right) to reset its odometry position by measuring distances to field walls. This provides accurate absolute positioning without relying on encoder accumulation errors.

### Sensor Configuration
The system uses three distance sensors mounted on the robot:

```
Robot Top View:
        ↑ (Front)
        │
    ┌───┼───┐
    │   │   │
    │   R   │  ← Robot
    │       │
    └───┼───┘
        │
        
Sensor Positions:
  Front:  PHI_FRONT = 0°    (points forward)
  Right:  PHI_RIGHT = 90°   (points right)
  Left:   PHI_LEFT = -90°    (points left)
```

**Visual Sensor Layout:**
```
        Front Sensor (0°)
            ↑
            │
    ┌───────┼───────┐
    │       │       │
Left│       R       │Right
(-90°)     │      (90°)
    │       │       │
    └───────┼───────┘
            │
         Robot Center
```

### Field Coordinate System
The field uses a standard coordinate system with walls at known positions:

```
Field Layout:
  Y
  ↑
  │
72│───────────────────  North Wall (Y = 72)
  │                   │
  │                   │
  │      Field        │
  │                   │
  │                   │
-72│───────────────────  South Wall (Y = -72)
  │
  └──────────────────────→ X
 -72                   72
West Wall              East Wall
(X = -72)              (X = 72)
```

### Relocalization Process

The `relocalize()` function calculates the robot's position using sensor readings and known wall positions.

**Step 1: Read Sensor Values**
```cpp
double dFront = readSensor(frontDistanceSensor);  // mm → inches
double dRight = readSensor(rightDistanceSensor);
double dLeft  = readSensor(leftDistanceSensor);
```

**Step 2: Determine Which Walls to Use**
The function accepts a string parameter indicating which walls are visible:
- `"N"` - North wall (Y = 72)
- `"S"` - South wall (Y = -72)
- `"E"` - East wall (X = 72)
- `"W"` - West wall (X = -72)
- Combinations: `"NW"`, `"SE"`, `"NSE"`, etc.

**Step 3: Calculate Y Coordinate (North/South)**
```
If using North or South wall:
  1. Get wall Y coordinate (72 or -72)
  2. Project front sensor distance onto global Y-axis
  3. Calculate: Y_robot = Y_wall - projection_Y
```

**Visual Example - North Wall:**
```
        North Wall (Y = 72)
        ───────────────────
              │
              │ dFront
              │
              ↓
        ┌─────┼─────┐
        │     R     │  Robot at heading θ
        └───────────┘
              │
              │
        projection_Y = dFront × sin(θ + 0°)
        
        Y_robot = 72 - projection_Y
```

**Step 4: Calculate X Coordinate (East/West)**
```
If using East or West wall:
  1. Get wall X coordinate (72 or -72)
  2. Find best sensor (front, left, or right) pointing toward wall
  3. Project sensor distance onto global X-axis
  4. Calculate: X_robot = X_wall - projection_X
```

**Visual Example - East Wall:**
```
East Wall (X = 72)
│
│  dRight
│    ←───┐
│        │
│    ┌───┼───┐
│    │   R   │  Robot at heading θ
│    └───────┘
│        │
│        │
│  projection_X = dRight × cos(θ + 90°)
│
X_robot = 72 - projection_X
```

### Projection Mathematics

The `computeProjection()` function projects sensor distances onto global axes:

**For X-coordinate:**
```
projection_X = distance × cos(θ + φ)
```
Where:
- `distance`: Sensor reading (inches)
- `θ`: Robot heading (degrees)
- `φ`: Sensor angle offset (0°, 90°, -90°)

**For Y-coordinate:**
```
projection_Y = distance × sin(θ + φ)
```

**Visual Projection:**
```
Global Coordinate System:
        Y ↑
          │
          │    Sensor reading (d)
          │      ╱
          │     ╱
          │    ╱ angle = θ + φ
          │   ╱
          │  ╱
          │ ╱
          └──────────────→ X
          
Projection onto X: d × cos(θ + φ)
Projection onto Y: d × sin(θ + φ)
```

### Sensor Selection Logic

For X-coordinate calculation, the system selects the best sensor:

**East Wall (X = 72):**
1. Start with right sensor (points toward +X)
2. Check if front sensor has positive X projection
3. Check if left sensor has positive X projection
4. Use sensor with smallest positive projection (closest to wall)

**West Wall (X = -72):**
1. Start with left sensor (points toward -X)
2. Check if front sensor has negative X projection
3. Check if right sensor has negative X projection
4. Use sensor with smallest negative projection (closest to wall)

**Visual Sensor Selection:**
```
East Wall Scenario:
  ┌─────────────────┐
  │                 │
  │  ┌───┐          │
  │  │ R │          │  ← Robot
  │  └───┘          │
  │   ↑  ↑  ↑       │
  │   L  F  R       │  All sensors can see wall
  │                 │  System picks closest
  └─────────────────┘
```

### Complete Example

**Scenario: Robot near Northwest corner**

```
Field View:
        Y
        ↑
      72│───────────────────
        │                   │
        │  ┌───┐            │
        │  │ R │            │  Robot
        │  └───┘            │
        │                   │
      -72│───────────────────
        └────────────────────→ X
      -72                  72
```

**Sensor Readings:**
- Front: 24 inches (to North wall)
- Right: 48 inches (to East wall)
- Left: 12 inches (to West wall)
- Heading: 45° (pointing Northeast)

**Calculation:**
```
Using "NW" (North + West walls):

Y-coordinate (North wall):
  projection_Y = 24 × sin(45° + 0°) = 24 × 0.707 = 16.97"
  Y_robot = 72 - 16.97 = 55.03"

X-coordinate (West wall):
  Best sensor: Left (closest to West wall)
  projection_X = 12 × cos(45° + (-90°)) = 12 × cos(-45°) = 8.49"
  X_robot = -72 - (-8.49) = -63.51"
  
Final Position: (-63.51, 55.03)
```

### Implementation Code

```cpp
void relocalize(std::string walls) {
  double headingDeg = normalizeTarget(getInertialHeading());
  
  // Read all three sensors
  double dFront = readSensor(frontDistanceSensor);
  double dRight = readSensor(rightDistanceSensor);
  double dLeft  = readSensor(leftDistanceSensor);
  
  // Calculate Y from North/South wall
  if (walls.find('N') != std::string::npos) {
    double projY = computeProjection(dFront, headingDeg, PHI_FRONT, false);
    p.y = 72 - projY;  // North wall at Y = 72
  }
  else if (walls.find('S') != std::string::npos) {
    double projY = computeProjection(dFront, headingDeg, PHI_FRONT, false);
    p.y = -72 + projY;  // South wall at Y = -72
  }
  
  // Calculate X from East/West wall
  if (walls.find('E') != std::string::npos) {
    // Find best sensor pointing toward East wall
    // Project and calculate X
    p.x = 72 - projX;
  }
  else if (walls.find('W') != std::string::npos) {
    // Find best sensor pointing toward West wall
    // Project and calculate X
    p.x = -72 + projX;
  }
  
  resetOdometry(p.x, p.y);  // Reset odometry to calculated position
}
```

### Usage Examples

**Reset using North and West walls:**
```cpp
relocalize("NW");  // Uses front sensor for Y, left sensor for X
```

**Reset using South and East walls:**
```cpp
relocalize("SE");  // Uses front sensor for Y, right sensor for X
```

**Reset using all walls (most accurate):**
```cpp
relocalize("NSEW");  // Uses best sensors for both X and Y
```

### Advantages

- **Absolute positioning**: No drift from encoder accumulation
- **Quick reset**: Instant position update
- **Flexible**: Works with any combination of visible walls
- **Accurate**: Typically within 1-2 inches of actual position
- **Robust**: Multiple sensor options provide redundancy

### Limitations

- **Requires walls**: Must be near field boundaries
- **Sensor range**: Distance sensors have limited range (~2000mm)
- **Angle dependency**: Accuracy depends on robot heading
- **Obstructions**: Objects between robot and wall affect readings

### Visual Summary

```
┌─────────────────────────────────────────┐
│  Relocalization Process Flow            │
└─────────────────────────────────────────┘
         │
         ▼
┌─────────────────────┐
│ Read 3 Sensors      │
│ - Front (0°)        │
│ - Right (90°)       │
│ - Left (-90°)       │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Determine Walls     │
│ (N/S/E/W)          │
└──────────┬──────────┘
           │
           ├──────────────┐
           │              │
           ▼              ▼
┌─────────────────┐  ┌─────────────────┐
│ Calculate Y     │  │ Calculate X     │
│ (North/South)   │  │ (East/West)     │
│ - Project front │  │ - Select best   │
│   sensor        │  │   sensor        │
│ - Y = wall -    │  │ - Project onto  │
│   projection    │  │   X-axis        │
└────────┬────────┘  └────────┬────────┘
         │                    │
         └──────────┬─────────┘
                    ▼
         ┌─────────────────────┐
         │ Reset Odometry      │
         │ x_pos = X           │
         │ y_pos = Y           │
         └─────────────────────┘
```

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

