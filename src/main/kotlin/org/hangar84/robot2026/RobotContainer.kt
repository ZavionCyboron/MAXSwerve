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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.subsystems.DriveSubsystem


/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
object RobotContainer {

    private const val DriveDeadband: Double = 0.05

    // The robot's subsystems
    private val robotDrive = DriveSubsystem

    // The driver's controller
    private val controller: CommandXboxController = CommandXboxController(0)

    var autoChooser : SendableChooser<Command>? = null

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
                throttleX = -MathUtil.applyDeadband(controller.leftY, DriveDeadband),
                throttleY = -MathUtil.applyDeadband(controller.leftX, DriveDeadband),
                throttleAngular = -MathUtil.applyDeadband(controller.rightX, DriveDeadband),
                fieldRelative = true,
            )

        // Park on left bumper
        controller.leftBumper().whileTrue(DriveSubsystem.park())
    }
}