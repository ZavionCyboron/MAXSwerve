package org.hangar84.robot2026

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.hangar84.robot2026.constants.Constants.ControllerConstants.JOYSTICK_DEADBAND
import org.hangar84.robot2026.constants.Constants.ControllerConstants.controller
import org.hangar84.robot2026.subsystems.DriveSubsystem
import org.hangar84.robot2026.subsystems.LauncherSubsystem

object RobotContainer {
    private var autoChooser: SendableChooser<Command>? = null

    val autonomousCommand: Command
        get() = autoChooser?.selected ?: InstantCommand()

    init {
        configureBindings()

        AutoBuilder.configure(
            // poseSupplier =
            { DriveSubsystem.poseEstimator.estimatedPosition },
            // resetPose =
            DriveSubsystem.poseEstimator::resetPose,
            // IntelliJ is off its rocker here. The spread operator works here, is practically required, and compiles.
            // The following error should be ignored, since there is no way to remove/hide it.
            // robotRelativeSpeedsSupplier =
            { DriveSubsystem.kinematics.toChassisSpeeds(*DriveSubsystem.allModuleStates) },
            // output =
            DriveSubsystem::driveRelative,
            // controller =
            PPHolonomicDriveController(
                // translationConstants =
                PIDConstants(5.0, 0.0, 0.0),
                // rotationConstants =
                PIDConstants(5.0, 0.0, 0.0),
            ),
            // robotConfig =
            try {
                RobotConfig.fromGUISettings()
            } catch (_: Exception) {
                null
            },
            // shouldFlipPath =
            { DriverStation.getAlliance()?.get() == DriverStation.Alliance.Red },
            // ...driveRequirements =
            DriveSubsystem,
        )

        autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Autonomous Routine", autoChooser)
    }

    private fun configureBindings() {
        DriveSubsystem.defaultCommand =
            DriveSubsystem.driveControlled(
                throttleX = -MathUtil.applyDeadband(controller.leftY, JOYSTICK_DEADBAND),
                throttleY = -MathUtil.applyDeadband(controller.leftX, JOYSTICK_DEADBAND),
                throttleAngular = -MathUtil.applyDeadband(controller.rightX, JOYSTICK_DEADBAND),
                fieldRelative = true,
            )
        LauncherSubsystem.defaultCommand =
            LauncherSubsystem.launcherControlled(
                launch = controller.leftTriggerAxis,
                intake = -controller.leftTriggerAxis
            )

        // Park on left bumper
        controller.leftBumper().whileTrue(DriveSubsystem.park())
    }
}