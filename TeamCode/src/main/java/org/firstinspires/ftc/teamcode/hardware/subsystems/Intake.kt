package org.firstinspires.ftc.teamcode.hardware.subsystems

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import java.time.OffsetDateTime

class Intake(hardwareMap: HardwareMap): Subsystem() {
    val left: Servo = hardwareMap.get("I.L") as Servo
    val right: Servo = hardwareMap.get("I.R") as Servo

    var state: State = State.OFF
    set(value) {
        field = value
        when (value) {
            State.OFF -> {
                left.position = 0.5
                right.position = 0.5
            }
            State.IN -> {
                left.position = 0.1
                right.position = 0.9
            }
            State.OUT -> {
                left.position = 0.9
                right.position = 0.1
            }
        }
    }

    enum class State {
        IN,
        OUT,
        OFF
    }
}