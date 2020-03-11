package org.firstinspires.ftc.teamcode.hardware.subsystems

import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor

interface Subsystem {
    val motors: List<OptimizedMotor>
    var lastUpdateTime: Long
}