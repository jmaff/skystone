package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.motion.wrapAngle
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import kotlin.math.hypot

@TeleOp(name = "Measure Slip")
class MeasureSlip: RobotOpMode() {
    val FORWARDS_SPEED = 0.3
    val LATERAL_SPEED = 0.3
    val TURN_SPEED = 0.4

    val ACCELERATION_TIME = 1000
    val SLIP_TIME = 1000

    var endSpeedY = 0.0
    var endSpeedX = 0.0
    var endSpeedTurn = 0.0

    var stateStartTime = 0L

    var xSlip = 0.0
    var ySlip = 0.0
    var turnSlip = 0.0

    enum class States {
        CONTROL_1,
        MOVE_Y,
        MEASURE_Y_SLIP,
        CONTROL_2,
        MOVE_X,
        MEASURE_X_SLIP,
        CONTROL_3,
        TURN,
        MEASURE_TURN_SLIP,
        END
    }

    var startX = 0.0
    var startY = 0.0
    var startAngle = 0.0

    var state = States.CONTROL_1

    fun initState() {
        stateStartTime = System.currentTimeMillis()
        startX = drivetrain.odometer.xPosition
        startY = drivetrain.odometer.yPosition
        startAngle = drivetrain.odometer.angle
    }

    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(0.0, 0.0, 0.0)
    }

    override fun loop() {
        super.loop()
        when (state) {
            States.CONTROL_1, States.CONTROL_2, States.CONTROL_3 -> {
                drivetrain.xPower = gamepad1.left_stick_x.toDouble()
                drivetrain.yPower = -gamepad1.left_stick_y.toDouble()
                drivetrain.turnPower = -gamepad1.right_stick_x.toDouble()

                if (gamepad1.a) {
                    stopMovement()
                    state = when (state) {
                        States.CONTROL_1 -> States.MOVE_Y
                        States.CONTROL_2 -> States.MOVE_X
                        States.CONTROL_3 -> States.TURN
                        else -> States.END
                    }
                    initState()
                }
            }
            States.MOVE_Y -> {
                drivetrain.yPower = FORWARDS_SPEED
                if (System.currentTimeMillis() - stateStartTime > ACCELERATION_TIME) {
                    endSpeedY = drivetrain.odometer.ySpeed
                    stopMovement()
                    state = States.MEASURE_Y_SLIP
                    initState()
                }
            }
            States.MEASURE_Y_SLIP -> {
                val dist = hypot((drivetrain.odometer.xPosition - startX), drivetrain.odometer.yPosition - startY)
                if (System.currentTimeMillis() - stateStartTime > SLIP_TIME) {
                    ySlip = dist / endSpeedY
                    stopMovement()
                    telemetry.addData("Y", ySlip)
                    state = States.CONTROL_2
                }
            }
            States.MOVE_X -> {
                drivetrain.xPower = LATERAL_SPEED
                if (System.currentTimeMillis() - stateStartTime > ACCELERATION_TIME) {
                    endSpeedX = drivetrain.odometer.xSpeed
                    stopMovement()
                    state = States.MEASURE_X_SLIP
                    initState()
                }
            }
            States.MEASURE_X_SLIP -> {
                val dist = hypot((drivetrain.odometer.xPosition - startX), drivetrain.odometer.yPosition - startY)
                if (System.currentTimeMillis() - stateStartTime > SLIP_TIME) {
                    xSlip = dist / endSpeedX
                    stopMovement()
                    telemetry.addData("X", xSlip)
                    state = States.CONTROL_3
                }
            }
            States.TURN -> {
                drivetrain.turnPower = TURN_SPEED
                if (System.currentTimeMillis() - stateStartTime > ACCELERATION_TIME) {
                    endSpeedTurn = drivetrain.odometer.angularVelocity
                    stopMovement()
                    state = States.MEASURE_TURN_SLIP
                    initState()
                }
            }
            States.MEASURE_TURN_SLIP -> {
                val turned = wrapAngle(drivetrain.odometer.angle - startAngle)
                if (System.currentTimeMillis() - stateStartTime > SLIP_TIME) {
                    turnSlip = turned / endSpeedTurn
                    stopMovement()
                    telemetry.addData("TURN", turnSlip)
                    state = States.END
                }
            }
            States.END -> {
                telemetry.addData("X", xSlip)
                telemetry.addData("Y", ySlip)
                telemetry.addData("TURN", turnSlip)
            }
        }
    }


}