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
    // front up: 0, front down: 1
    val FRONT_UP = 0.0
    val FRONT_DOWN = 0.9

    val BACK_UP = 1.0
    val BACK_DOWN = 0.0

    val FRONT_READY = FRONT_UP
    val BACK_READY = 0.1
    val FRONT_GRABBED = FRONT_DOWN
    val BACK_GRABBED = BACK_DOWN
    val FRONT_RELEASED = FRONT_UP
    val BACK_RELEASED = BACK_UP
    val PIVOT_REGULAR = 0.04
    val PIVOT_PERPENDICULAR = 0.5

    val FOUR_BAR_POWER = 1.0
    val READY_POSITION = 0
    val GRAB_POSITION = 180
    val OUT_POSITION = 650
    val DECELERATION_INTERVAL = 400 // in encoder counts

    val fourBar = OptimizedMotor(hardwareMap.get("T.Fb") as ExpansionHubMotor, false)
    override val motors = listOf(fourBar)

    init {
        fourBar.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        fourBar.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        fourBar.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    val backGrab = hardwareMap.get("T.B") as Servo
    val frontGrab = hardwareMap.get("T.F") as Servo
    val pivot = hardwareMap.get("T.P") as Servo

    enum class GrabberState {
        READY_FOR_STONE,
        GRABBED,
        PERPENDICULAR,
        RELEASED,
        RELEASED_PERP
    }

    var grabberState = GrabberState.READY_FOR_STONE
    set(value) {
        field = value
        when (value) {
            GrabberState.READY_FOR_STONE -> {
                pivot.position = PIVOT_REGULAR
                backGrab.position = BACK_READY
                frontGrab.position = FRONT_READY
            }
            GrabberState.GRABBED -> {
                pivot.position = PIVOT_REGULAR
                backGrab.position = BACK_GRABBED
                frontGrab.position = FRONT_GRABBED
            }
            GrabberState.PERPENDICULAR -> {
                pivot.position = PIVOT_PERPENDICULAR
                backGrab.position = BACK_GRABBED
                frontGrab.position = FRONT_GRABBED
            }
            GrabberState.RELEASED -> {
                pivot.position = PIVOT_REGULAR
                backGrab.position = BACK_RELEASED
                frontGrab.position = FRONT_RELEASED
            }
            GrabberState.RELEASED_PERP -> {
                pivot.position = PIVOT_PERPENDICULAR
                backGrab.position = BACK_RELEASED
                frontGrab.position = FRONT_RELEASED
            }
        }
    }

    enum class FourBarPosition {
        LONG_DOWN,
        SHORT_UP,
        LONG_UP,
        READY,
        GRAB,
        OUT
    }

    var prev = FourBarPosition.READY

    var fourBarPosition = FourBarPosition.LONG_DOWN
    set(value) {
        field = value
        when (value) {
            FourBarPosition.LONG_DOWN -> {
                if (!running) {
                    runFourBarFor(1100, -0.85)
                }
            }
            FourBarPosition.LONG_UP -> {
                if (!running) {
                    runFourBarFor(1100, 0.85)
                }
            }
            FourBarPosition.SHORT_UP -> {
                if (!running) {
                    runFourBarFor(200, 1.0)
                }
            }
        }

    }
//    var prev = FourBarPosition.READY

    var running = false
    var startTime = 0L
    var fourBarPower = 0.0
    var time = 0L

    fun runFourBarFor(time: Long, power: Double) {
        running = true
        startTime = System.currentTimeMillis()
        this.time = time
        fourBarPower = power
    }

    fun update() {
        if (running) {
            if (System.currentTimeMillis() - startTime >= time) {
                fourBar.power = 0.0
                running = false
            } else {
                fourBar.power = fourBarPower
            }
        }
    }

    var automatic = false

    fun runFourBarTo(position: FourBarPosition): Boolean {
        if (automatic) {
            val error = when (position) {
                FourBarPosition.READY -> READY_POSITION - fourBar.currentPosition
                FourBarPosition.GRAB -> GRAB_POSITION - fourBar.currentPosition
                FourBarPosition.OUT -> OUT_POSITION - fourBar.currentPosition
                else -> 0
            }

            val decelerationInterval: Int = when (position) {
                FourBarPosition.OUT -> 300
                FourBarPosition.READY -> {
                    if (prev == FourBarPosition.OUT) {
                        250
                    } else {
                        200
                    }
                }
                FourBarPosition.GRAB -> {
                    if (prev == FourBarPosition.READY) {
                        50
                    } else {
                        150
                    }
                }
                else -> 0
            }

            val power = error.toDouble() / decelerationInterval.toDouble()
            fourBar.power = Range.clip(power, -FOUR_BAR_POWER, FOUR_BAR_POWER)
            return abs(error) < 4
        }
        return true
    }
}