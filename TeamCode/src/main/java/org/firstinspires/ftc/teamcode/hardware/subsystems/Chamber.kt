package org.firstinspires.ftc.teamcode.hardware.subsystems

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Chamber(hardwareMap: HardwareMap): Subsystem() {
    val orient = hardwareMap.get("C.O") as Servo
    val leftDist = hardwareMap.get("C.L") as DistanceSensor
    val rightDist = hardwareMap.get("C.R") as DistanceSensor


    enum class StoneOrientation {
        PEGS_LEFT,
        PEGS_RIGHT,
        NORMAL
    }

    var stoneOrientation: StoneOrientation = StoneOrientation.NORMAL
        set(value) {
            field = value
            when (value) {
                StoneOrientation.PEGS_LEFT -> orient.position = 0.6
                StoneOrientation.PEGS_RIGHT -> orient.position = 0.4
                StoneOrientation.NORMAL -> orient.position = 0.5
            }
//            orient.position = when (value) {
//                StoneOrientation.NORMAL -> 0.5
//                StoneOrientation.PEGS_LEFT -> 0.7
//                StoneOrientation.PEGS_RIGHT -> 0.3
//            }
        }
}