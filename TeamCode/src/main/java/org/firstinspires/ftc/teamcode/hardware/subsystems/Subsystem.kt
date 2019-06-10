package org.firstinspires.ftc.teamcode.hardware.subsystems

import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor

abstract class Subsystem {
    abstract val motors: List<OptimizedMotor>
    var lastUpdateTime: Long = 0
}