package org.firstinspires.ftc.teamcode.hardware.subsystems

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.openftc.revextensions2.ExpansionHubMotor
import java.time.OffsetDateTime

class Intake(hardwareMap: HardwareMap): Subsystem() {
    val motor: OptimizedMotor = OptimizedMotor(hardwareMap.get("I.M") as ExpansionHubMotor, false)
    override val motors: List<OptimizedMotor> = listOf(motor)

    var state: State = State.OFF
    set(value) {
        field = value
        when (value) {
            State.OFF -> {
                motor.power = 0.0
            }
            State.IN -> {
                motor.power = 0.85
            }
            State.OUT -> {
                motor.power = -0.85
            }
        }
    }

    enum class State {
        IN,
        OUT,
        OFF
    }
}