package org.hangar84.robot2026.subsystems

import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.hangar84.robot2026.constants.Constants.LauncherConstants.LAUNCHER_MAX_SPEED
import org.hangar84.robot2026.constants.Constants.LauncherConstants.launcherFollowerMotor
import org.hangar84.robot2026.constants.Constants.LauncherConstants.launcherMotor

object LauncherSubsystem : Subsystem {

    val launcherMotorConfig = SparkMaxConfig()
    val launcherFollowerMotorConfig = SparkMaxConfig()


    init {
        launcherFollowerMotorConfig.follow(launcherFollowerMotor)
        launcherFollowerMotorConfig.follow(launcherMotor)
        launcherMotorConfig.inverted(true)
    }

    fun launcherControlled(
        launch: Double,
        intake: Double
    ): Command =
        this.run{
            val launch = LAUNCHER_MAX_SPEED * launch
            val intake = LAUNCHER_MAX_SPEED * intake
    }
}