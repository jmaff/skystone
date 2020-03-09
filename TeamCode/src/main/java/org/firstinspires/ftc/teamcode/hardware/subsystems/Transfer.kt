package org.firstinspires.ftc.teamcode.hardware.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.openftc.revextensions2.ExpansionHubMotor
import kotlin.math.abs

class Transfer(hardwareMap: HardwareMap): Subsystem() {
    val PIVOT_REGULAR = 0.4635 // 1427
    val PIVOT_LEFT = 0.7895 // // 2079
    val PIVOT_RIGHT = 0.1555 // 811

    val GRABBED = 0.638 // 500
    val RELEASED = 1.00 // 1114

    val CAP_HOLD = 0.803 // 2106
    val CAP_RELEASE = 0.2 // 500

    val BAR_READY = 0.72 // 1925; 1917
    val BAR_GRAB = 0.80 // 2022; 2026
    val BAR_PEG_ALIGN = 0.4135 // 1366; 1327
    val BAR_DOWN = 0.319 // 1063; 1138
    val BAR_MAX_DOWN = 0.2

    val bar = hardwareMap.get("T.B") as Servo
    val grab = hardwareMap.get("T.G") as Servo
    val pivot = hardwareMap.get("T.P") as Servo
    val capstone = hardwareMap.get("T.C") as Servo

    enum class GrabberState {
        READY_FOR_STONE,
        GRABBED,
        RELEASED
    }

    var grabberState = GrabberState.READY_FOR_STONE
    set(value) {
        field = value
        when (value) {
            GrabberState.READY_FOR_STONE -> {
                pivot.position = PIVOT_REGULAR
                grab.position = RELEASED
            }
            GrabberState.GRABBED -> {
                pivot.position = PIVOT_REGULAR
                grab.position = GRABBED
            }
            GrabberState.RELEASED -> {
                grab.position = RELEASED
            }
        }
    }

    enum class FourBarPosition {
        READY,
        GRAB,
        PEG_ALIGN,
        DOWN,
        MAX_DOWN
    }

    var fourBarPosition = FourBarPosition.READY
    set(value) {
        field = value
        bar.position = when (value) {
            FourBarPosition.READY -> BAR_READY
            FourBarPosition.GRAB -> BAR_GRAB
            FourBarPosition.PEG_ALIGN -> BAR_PEG_ALIGN
            FourBarPosition.DOWN -> BAR_DOWN
            FourBarPosition.MAX_DOWN -> BAR_MAX_DOWN
        }
    }

    fun update() {

    }
}