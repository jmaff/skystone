package org.firstinspires.ftc.teamcode.hardware.subsystems

import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor

abstract class Subsystem {
    open val motors: List<OptimizedMotor> = listOf()
    var lastUpdateTime: Long = 0
}