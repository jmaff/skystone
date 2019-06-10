package org.firstinspires.ftc.teamcode.hardware.devices

class Encoder(val delegateMotor: OptimizedMotor) {
    val counts
        get() = delegateMotor.currentPosition
}