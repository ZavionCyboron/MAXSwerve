package org.hangar84.robot2026.constants

import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.swerve.MAXSwerveModule
import org.hangar84.robot2026.swerve.SwerveConfigs

object Constants {

    object SwerveConfigsConstants {
        // -- SwerveConfigs Constants --
        const val DRIVE_PINION_TEETH = 12
        const val NEO_FREE_SPEED_RPM = 5676
        const val WHEEL_DIAMETER = 0.0762
        const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI
        const val DRIVING_REDUCTION = (45 * 22) / (DRIVE_PINION_TEETH * 15)
        const val DRIVE_WHEEL_FREE_SPEED_RPS = (NEO_FREE_SPEED_RPM / 60) * WHEEL_CIRCUMFERENCE / DRIVING_REDUCTION
        const val DRIVING_FACTOR = WHEEL_CIRCUMFERENCE / DRIVING_REDUCTION
        const val TURNING_FACTOR = 2 * Math.PI
        const val DRIVING_FEEDFOWARD = 1.0 / DRIVE_WHEEL_FREE_SPEED_RPS
    }

    object DriveConstants {
        // -- DriveSubsystem Constants --
        val MAX_SPEED = MetersPerSecond.of(4.8)
        val MAX_ANGULAR_SPEED = RotationsPerSecond.of(1.0)
        val WHEEL_BASE = Inches.of(24.0)
        val TRACK_WIDTH = Inches.of(24.5)

        // -- Module Constants --
        val frontRightModule = MAXSwerveModule(
            1,2,Degrees.of(0.0),
            SwerveConfigs.drivingConfig, SwerveConfigs.turningConfig
        )

        val frontLeftModule = MAXSwerveModule(
            3,4, Degrees.of(270.0),
            SwerveConfigs.drivingConfig, SwerveConfigs.turningConfig
        )

        val rearRightModule = MAXSwerveModule(
            5,6,Degrees.of(90.0),
            SwerveConfigs.drivingConfig, SwerveConfigs.turningConfig
        )

        val rearLeftModule = MAXSwerveModule(
            7,8, Degrees.of(180.0),
            SwerveConfigs.drivingConfig, SwerveConfigs.turningConfig
        )

    }

    object ControllerConstants {
        const val JOYSTICK_DEADBAND = 0.05

        val controller = CommandXboxController(0)
    }

    object LauncherConstants {
        val LAUNCHER_MAX_SPEED = MetersPerSecond.of(4.8)

        //val launcherMotor = SparkMax(9, MotorType.kBrushless)
        //val launcherFollowerMotor = SparkMax(10, MotorType.kBrushless)
    }


}