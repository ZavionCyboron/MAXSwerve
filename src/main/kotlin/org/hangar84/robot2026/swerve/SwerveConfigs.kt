package org.hangar84.robot2026.swerve

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.Constants.SwerveConfigsConstants.DRIVING_FACTOR
import org.hangar84.robot2026.constants.Constants.SwerveConfigsConstants.TURNING_FACTOR
import org.hangar84.robot2026.constants.Constants.SwerveConfigsConstants.DRIVING_FEEDFOWARD


object SwerveConfigs {
    val drivingConfig = SparkMaxConfig()

    val turningConfig = SparkMaxConfig()

    init {
        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)

        drivingConfig.encoder
            .positionConversionFactor(DRIVING_FACTOR)
            .velocityConversionFactor(DRIVING_FACTOR / 60)

        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid( 0.04, 0.0, 0.0) // TODO: Tune these values (if needed)
            .velocityFF(DRIVING_FEEDFOWARD)
            .outputRange(-1.0, 1.0)

        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)

        turningConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(TURNING_FACTOR)
            .velocityConversionFactor(TURNING_FACTOR / 60)

        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0,0.0,0.0) // TODO: Tune these values (if needed)
            .outputRange(-1.0, 1.0)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, TURNING_FACTOR)
    }
}