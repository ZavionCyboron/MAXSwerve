package org.hangar84.robot2026.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
    object DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        const val MaxSpeedMetersPerSecond: Double = 4.8
        val MaxAngularSpeed: Double = 2 * Math.PI // radians per second

        // Chassis configuration
        val TrackWidth: Double = Units.inchesToMeters(26.5)

        // Distance between centers of right and left wheels on robot
        val WheelBase: Double = Units.inchesToMeters(26.5)

        // Distance between front and back wheels on robot
        val DriveKinematics: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(WheelBase / 2, TrackWidth / 2),
            Translation2d(WheelBase / 2, -TrackWidth / 2),
            Translation2d(-WheelBase / 2, TrackWidth / 2),
            Translation2d(-WheelBase / 2, -TrackWidth / 2)
        )

        // Angular offsets of the modules relative to the chassis in radians
        val FrontLeftChassisAngularOffset: Double = -Math.PI / 2
        const val FrontRightChassisAngularOffset: Double = 0.0
        val BackLeftChassisAngularOffset: Double = Math.PI
        val BackRightChassisAngularOffset: Double = Math.PI / 2

        // SPARK MAX CAN IDs
        const val FrontLeftDrivingCanId: Int = 11
        const val RearLeftDrivingCanId: Int = 13
        const val FrontRightDrivingCanId: Int = 15
        const val RearRightDrivingCanId: Int = 17

        const val FrontLeftTurningCanId: Int = 10
        const val RearLeftTurningCanId: Int = 12
        const val FrontRightTurningCanId: Int = 14
        const val RearRightTurningCanId: Int = 16

        const val GyroReversed: Boolean = false
    }

    object ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        const val DrivingMotorPinionTeeth: Int = 14

        // Calculations required for driving motor conversion factors and feed forward
        val DrivingMotorFreeSpeedRps: Double = NeoMotorConstants.FreeSpeedRpm / 60
        const val WheelDiameterMeters: Double = 0.0762
        val WheelCircumferenceMeters: Double = WheelDiameterMeters * Math.PI

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        val DrivingMotorReduction: Double = (45.0 * 22) / (DrivingMotorPinionTeeth * 15)
        val DriveWheelFreeSpeedRps: Double = ((DrivingMotorFreeSpeedRps * WheelCircumferenceMeters)
                / DrivingMotorReduction)
    }

    object OIConstants {
        const val DriverControllerPort: Int = 0
        const val DriveDeadband: Double = 0.05
    }

    object AutoConstants {
        const val MaxSpeedMetersPerSecond: Double = 3.0
        const val MaxAccelerationMetersPerSecondSquared: Double = 3.0
        val MaxAngularSpeedRadiansPerSecond: Double = Math.PI
        val MaxAngularSpeedRadiansPerSecondSquared: Double = Math.PI

        const val PXController: Double = 1.0
        const val PYController: Double = 1.0
        const val PThetaController: Double = 1.0

        // Constraint for the motion profiled robot angle controller
        val ThetaControllerConstraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
            MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared
        )
    }

    object NeoMotorConstants {
        const val FreeSpeedRpm: Double = 5676.0
    }
}