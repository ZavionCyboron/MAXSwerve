package org.hangar84.robot2026.swerve

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig

import org.hangar84.robot2026.constants.Constants.ModuleConstants


class Configs {
    object MAXSwerveModule {
        val drivingConfig: SparkMaxConfig = SparkMaxConfig()
        val turningConfig: SparkMaxConfig = SparkMaxConfig()

        init {
            // Use module constants to calculate conversion factors and feed forward gain.
            val drivingFactor = (ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction)
            val turningFactor = 2 * Math.PI
            val drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps

            drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
            drivingConfig.encoder
                .positionConversionFactor(drivingFactor) // meters
                .velocityConversionFactor(drivingFactor / 60.0) // meters per second
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // These are example gains you may need to them for your own robot!
                .pid(0.04, 0.0, 0.0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1.0, 1.0)

            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
            turningConfig.absoluteEncoder // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(true)
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0) // radians per second
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // These are example gains you may need to them for your own robot!
                .pid(1.0, 0.0, 0.0)
                .outputRange(-1.0, 1.0) // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0.0, turningFactor)
        }
    }
}