package org.hangar84.robot2026.subsystems

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkClosedLoopController
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.hangar84.robot2026.swerve.Configs

class MAXSwerveModule(drivingCANId: Int, turningCANId: Int, chassisAngularOffset: Double) {
    private val drivingSpark: SparkMax = SparkMax(drivingCANId, SparkLowLevel.MotorType.kBrushless)
    private val turningSpark: SparkMax = SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless)

    private val drivingEncoder: RelativeEncoder = drivingSpark.getEncoder()
    private val turningEncoder = turningSpark.getAbsoluteEncoder()

    private val drivingClosedLoopController: SparkClosedLoopController = drivingSpark.getClosedLoopController()
    private val turningClosedLoopController: SparkClosedLoopController = turningSpark.getClosedLoopController()

    private var chassisAngularOffset = 0.0
    private var desiredState = SwerveModuleState(0.0, Rotation2d())

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    init {

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        drivingSpark.configure(
            Configs.MAXSwerveModule.drivingConfig, SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        turningSpark.configure(
            Configs.MAXSwerveModule.turningConfig, SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        this.chassisAngularOffset = chassisAngularOffset
        desiredState.angle = Rotation2d(turningEncoder.getPosition())
        drivingEncoder.position = 0.0
    }

    val position: SwerveModulePosition
        /**
         * Returns the current position of the module.
         *
         * @return The current position of the module.
         */
        get() =// Apply chassis angular offset to the encoder position to get the position
            // relative to the chassis.
            SwerveModulePosition(
                drivingEncoder.position,
                Rotation2d(turningEncoder.position - chassisAngularOffset)
            )

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleState) {
        // Apply chassis angular offset to the desired state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d(turningEncoder.getPosition()))

        // Command driving and turning SPARKS towards their respective setpoints.
        drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, SparkBase.ControlType.kVelocity)
        turningClosedLoopController.setReference(correctedDesiredState.angle.radians, SparkBase.ControlType.kPosition)

        this.desiredState = desiredState
    }

    /** Zeroes all the SwerveModule encoders.  */
    fun resetEncoders() {
        drivingEncoder.position = 0.0
    }
}