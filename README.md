# MAXSwerve

Kotlin WPILib **swerve drivetrain** project (REV MAXSwerve-style modules) used by
**Early College / Hangar 84** for drivetrain development, telemetry, and autonomous path following.

## Overview
This repo implements a swerve drivetrain with:
- odometry + pose estimation
- PhotonVision-based vision pose updates
- PathPlanner autonomous using **AutoBuilder**
- real-time telemetry (modules, chassis, pose)

## Key Features
- Kotlin + WPILib command-based structure
- Swerve module control + kinematics
- Odometry + `SwerveDrivePoseEstimator`
- Vision updates using **PhotonVision** AprilTags
- PathPlanner autonomous via **AutoBuilder**
- Telemetry to **SmartDashboard / NetworkTables**
- Simulation support (sensor simulation + pose comparison)

## Controls (Xbox Controller)
_(Typical mapping used in your RobotContainer projects)_
- Left stick: translation (X/Y)
- Right stick X: rotation
- Left bumper (hold): park/lock wheels (calls PARK command)

## Autonomous (PathPlanner + AutoBuilder)
Autonomous selection is built using `buildAutoChooser()` and PathPlanner’s `AutoBuilder`.
Paths can be flipped automatically based on alliance color. :contentReference[oaicite:0]{index=0}

## Telemetry & Monitoring
This project publishes detailed telemetry for debugging and tuning, including:

### High-level robot telemetry
- `Swerve/YawDeg`
- `Swerve/Pose`
- `Swerve/Chassis/*` (vx, vy, omega) :contentReference[oaicite:1]{index=1}

### Module telemetry (per module: FL, FR, RL, RR)
- `Swerve/<Module>/SpeedMps`, `AngleDeg`
- `Swerve/<Module>/DesiredSpeedMps`, `DesiredAngleDeg`
- Drive: Current / Output / Temp
- Turn: Current / Output / Temp :contentReference[oaicite:2]{index=2}

### Vision telemetry
- `Swerve/Vision/HasEstimate` :contentReference[oaicite:3]{index=3}

### Simulation telemetry
- `Swerve/Sim/Pose`, `Swerve/Sim/YawDeg`
- `Swerve/Sim/Cmd` (commanded chassis speeds)
- `Swerve/Sim/Encoders`, `Swerve/Sim/AzimuthDeg`
- `Swerve/Sim/PoseCompare` (ground truth vs estimated) :contentReference[oaicite:4]{index=4}

## Hardware
- IMU: ADIS16470 (real robot), simulated yaw in sim :contentReference[oaicite:5]{index=5}
- Vision: PhotonVision camera named `"FrontCamera"` :contentReference[oaicite:6]{index=6}
- CAN IDs in `Constants.Swerve` :contentReference[oaicite:7]{index=7}

## Contributing (Team Workflow)
- Branches: `feature/...` and `fix/...`
- Validate drive + telemetry in sim when possible
- Keep telemetry keys stable so dashboards don’t break

## Credits
Early College High School Robotics / Hangar 84  
Built with WPILib, PathPlanner, REV libraries, and PhotonVision.
