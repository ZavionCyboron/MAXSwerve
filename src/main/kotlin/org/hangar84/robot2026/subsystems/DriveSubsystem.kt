package org.hangar84.robot2026.subsystems

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.constants.Constants.DriveConstants


class DriveSubsystem : SubsystemBase() {
    // Create MAXSwerveModules
    private val frontLeft: MAXSwerveModule = MAXSwerveModule(
        DriveConstants.FrontLeftDrivingCanId,
        DriveConstants.FrontLeftTurningCanId,
        DriveConstants.FrontLeftChassisAngularOffset
    )

    private val frontRight: MAXSwerveModule = MAXSwerveModule(
        DriveConstants.FrontRightDrivingCanId,
        DriveConstants.FrontRightTurningCanId,
        DriveConstants.FrontRightChassisAngularOffset
    )

    private val rearLeft: MAXSwerveModule = MAXSwerveModule(
        DriveConstants.RearLeftDrivingCanId,
        DriveConstants.RearLeftTurningCanId,
        DriveConstants.BackLeftChassisAngularOffset
    )

    private val rearRight: MAXSwerveModule = MAXSwerveModule(
        DriveConstants.RearRightDrivingCanId,
        DriveConstants.RearRightTurningCanId,
        DriveConstants.BackRightChassisAngularOffset
    )

    // The gyro sensor
    private val gyro = ADIS16470_IMU()

    // Odometry class for tracking robot pose
    var odometry: SwerveDriveOdometry = SwerveDriveOdometry(
        DriveConstants.DriveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)),
        arrayOf<SwerveModulePosition?>(
            frontLeft.position,
            frontRight.position,
            rearLeft.position,
            rearRight.position
        )
    )

    /** Creates a new DriveSubsystem.  */
    init {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve)
    }

    override fun periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)),
            arrayOf<SwerveModulePosition?>(
                frontLeft.position,
                frontRight.position,
                rearLeft.position,
                rearRight.position
            )
        )
    }

    val pose: Pose2d?
        /**
         * Returns the currently-estimated pose of the robot.
         *
         * @return The pose.
         */
        get() = odometry.getPoseMeters()

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    fun resetOdometry(pose: Pose2d?) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)),
            arrayOf<SwerveModulePosition?>(
                frontLeft.position,
                frontRight.position,
                rearLeft.position,
                rearRight.position
            ),
            pose
        )
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     * field.
     */
    fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        // Convert the commanded speeds into the correct units for the drivetrain
        val xSpeedDelivered = xSpeed * DriveConstants.MaxSpeedMetersPerSecond
        val ySpeedDelivered = ySpeed * DriveConstants.MaxSpeedMetersPerSecond
        val rotDelivered = rot * DriveConstants.MaxAngularSpeed

        val swerveModuleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(
            if (fieldRelative)
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ))
                )
            else
                ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        )
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.MaxSpeedMetersPerSecond
        )
        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        rearLeft.setDesiredState(swerveModuleStates[2])
        rearRight.setDesiredState(swerveModuleStates[3])
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    fun setX() {
        frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.MaxSpeedMetersPerSecond
        )
        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        rearLeft.setDesiredState(desiredStates[2])
        rearRight.setDesiredState(desiredStates[3])
    }

    /** Resets the drive encoders to currently read a position of 0.  */
    fun resetEncoders() {
        frontLeft.resetEncoders()
        rearLeft.resetEncoders()
        frontRight.resetEncoders()
        rearRight.resetEncoders()
    }

    /** Zeroes the heading of the robot.  */
    fun zeroHeading() {
        gyro.reset()
    }

    val heading: Double
        /**
         * Returns the heading of the robot.
         *
         * @return the robot's heading in degrees, from -180 to 180
         */
        get() = Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)).getDegrees()

    val turnRate: Double
        /**
         * Returns the turn rate of the robot.
         *
         * @return The turn rate of the robot, in degrees per second
         */
        get() = gyro.getRate(IMUAxis.kZ) * (if (DriveConstants.GyroReversed) -1.0 else 1.0)
}