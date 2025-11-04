package org.hangar84.robot2026

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.hangar84.robot2026.constants.Constants.*
import org.hangar84.robot2026.subsystems.DriveSubsystem


/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
object RobotContainer {
    // The robot's subsystems
    private val robotDrive = DriveSubsystem()

    // The driver's controller
    var driverController: XboxController = XboxController(OIConstants.DriverControllerPort)

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        // Configure the button bindings
        configureButtonBindings()

        // Configure default commands
        robotDrive.defaultCommand = RunCommand(
            {
                robotDrive.drive(
                    -MathUtil.applyDeadband(driverController.leftY, OIConstants.DriveDeadband),
                    -MathUtil.applyDeadband(driverController.leftX, OIConstants.DriveDeadband),
                    -MathUtil.applyDeadband(driverController.rightX, OIConstants.DriveDeadband),
                    true
                )
            },
            robotDrive
        )
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a [edu.wpi.first.wpilibj.GenericHID] or one of its
     * subclasses ([ ] or [XboxController]), and then calling
     * passing it to a
     * [JoystickButton].
     */
    private fun configureButtonBindings() {
        JoystickButton(driverController, XboxController.Button.kRightBumper.value)
            .whileTrue(
                RunCommand(
                    { robotDrive.setX() },
                    robotDrive
                )
            )
    }

    val autonomousCommand: Command?
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() {
            // Create config for trajectory
            val config = TrajectoryConfig(
                AutoConstants.MaxSpeedMetersPerSecond,
                AutoConstants.MaxAccelerationMetersPerSecondSquared
            ) // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.DriveKinematics)

            // An example trajectory to follow. All units in meters.
            val exampleTrajectory: Trajectory =
                TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction
                    Pose2d(
                        0.0,
                        0.0,
                        Rotation2d(0.0)
                    ),  // Pass through these two interior waypoints, making an 's' curve path
                    listOf<Translation2d?>(
                        Translation2d(1.0, 1.0),
                        Translation2d(2.0, -1.0)
                    ),  // End 3 meters straight ahead of where we started, facing forward
                    Pose2d(3.0, 0.0, Rotation2d(0.0)),
                    config
                )

            val thetaController = ProfiledPIDController(
                AutoConstants.PThetaController, 0.0, 0.0, AutoConstants.ThetaControllerConstraints
            )
            thetaController.enableContinuousInput(-Math.PI, Math.PI)

            val swerveControllerCommand = SwerveControllerCommand(
                exampleTrajectory,
                robotDrive::pose,  // Functional interface to feed supplier
                DriveConstants.DriveKinematics,  // Position controllers

                PIDController(AutoConstants.PXController, 0.0, 0.0),
                PIDController(AutoConstants.PYController, 0.0, 0.0),
                thetaController,
                { desiredStates: Array<SwerveModuleState> -> robotDrive.setModuleStates(desiredStates) },
                robotDrive
            )

            // Reset odometry to the starting pose of the trajectory.
            robotDrive.resetOdometry(exampleTrajectory.initialPose)

            // Run path following command, then stop at the end.
            return swerveControllerCommand.andThen({ robotDrive.drive(0.0, 0.0, 0.0, false) })
        }
}


/*object RobotContainer {
    init {
        configureBindings()

        /*AutoBuilder.configure(
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
        SmartDashboard.putData("Autonomous Routine", autoChooser)*/
    }


    private fun configureBindings() {
        DriveSubsystem.defaultCommand =
            DriveSubsystem.driveControlled(
                throttleX = -MathUtil.applyDeadband(
                    controller.leftY,
                    JOYSTICK_DEADBAND
                ),
                throttleY = -MathUtil.applyDeadband(
                    controller.leftX,
                    JOYSTICK_DEADBAND
                ),
                throttleAngular = -MathUtil.applyDeadband(
                    controller.rightX,
                    JOYSTICK_DEADBAND
                ),
                fieldRelative = true,
            )

        // Park on left bumper
        controller.leftBumper().whileTrue(DriveSubsystem.park())
    }
}*/