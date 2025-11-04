package org.hangar84.robot2026

import edu.wpi.first.wpilibj.RobotBase

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
object Main {
    /**
     * Main initialization method. Do not perform any initialization here.
     *
     *
     * If you change your main Robot class (name), change the parameter type.
     */
    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot { Robot }
    }
}