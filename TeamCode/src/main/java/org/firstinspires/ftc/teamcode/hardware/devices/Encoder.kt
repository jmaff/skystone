package org.firstinspires.ftc.teamcode.hardware.devices

class Encoder(val delegateMotor: OptimizedMotor, val reversed: Boolean = false) {
    val counts
        get() = delegateMotor.currentPosition * if(reversed) -1 else 1
}