package org.firstinspires.ftc.teamcode.hardware.devices

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.openftc.revextensions2.ExpansionHubMotor
import kotlin.math.abs

class OptimizedMotor(private val delegateMotor: ExpansionHubMotor, val isMaster: Boolean) {
    val MIN_POWER_CHANGE = 0.005
    var currentPosition: Int = 0
    var lastPower: Double = 0.0
        private set
    var power: Double
        get() = delegateMotor.power
        set(value) {
             var toApply = value * powerScale
            if (abs(toApply - lastPower) > MIN_POWER_CHANGE ||
                (toApply == 0.0 && lastPower != 0.0)) {
                delegateMotor.power = toApply
                lastPower = toApply
            }
        }
    var mode: DcMotor.RunMode
        get() = delegateMotor.mode
        set(value) {
            delegateMotor.mode = value
        }
    var direction: DcMotorSimple.Direction
        get() = delegateMotor.direction
        set(value) {
            delegateMotor.direction = value
        }
    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
        get() = delegateMotor.zeroPowerBehavior
        set(value) {
            delegateMotor.zeroPowerBehavior = value
        }

    companion object {
        val powerScale = 1.0
    }


}