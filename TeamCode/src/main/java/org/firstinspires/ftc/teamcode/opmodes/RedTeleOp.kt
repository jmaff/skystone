package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.control.EnhancedGamepad
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystems.Transfer
import kotlin.math.PI

@TeleOp(name = "Red TeleOp")
class RedTeleOp: RobotOpMode() {
    lateinit var gamer1: EnhancedGamepad
    lateinit var gamer2: EnhancedGamepad

    var target = 0
    var prev = false
    var liftRunning = false
    val p = 0.01

    var strafing = false
    var angleLock = 0.0
    val turn_p = 0.01

    override fun start() {
        gamer1 = EnhancedGamepad(gamepad1)
        gamer2 = EnhancedGamepad(gamepad2)

        drivetrain.odometer.resetPosition(17 / 2 * 2.54, 17.75 / 2 * 2.54, PI / 2)
        transfer.fourBarPosition = Transfer.FourBarPosition.READY
        transfer.grabberState = Transfer.GrabberState.GRABBED
    }

    override fun loop() {
        super.loop()
        gamer1.update()
        gamer2.update()

        // foundation: 1
        if (gamer1.A.pressed && !drivetrain.foundationDown) {
            drivetrain.foundationDown = true
        } else if (gamer1.A.pressed && drivetrain.foundationDown) {
            drivetrain.foundationDown = false
        }

        // intake: 1
        if (gamer1.RIGHT_BUMPER.pressed) {
            if (intake.state != Intake.State.IN) {
                intake.state = Intake.State.IN
                transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
            } else {
                intake.state = Intake.State.OFF
            }
        }

        if (gamer1.LEFT_BUMPER.pressed) {
            if (intake.state != Intake.State.OUT) {
                intake.state = Intake.State.OUT
            } else {
                intake.state = Intake.State.OFF
            }
        }

        // drive: 1
        if (!strafing) {
            drivetrain.xPower = gamepad1.left_stick_x.toDouble()
            drivetrain.yPower = -gamepad1.left_stick_y.toDouble()
            drivetrain.turnPower = -gamepad1.right_stick_x.toDouble() * 0.7
        }

        // lift: 2
        if (gamer2.delegate.right_trigger > 0.0) {
            lift.power = -gamer2.delegate.right_trigger.toDouble()
            liftRunning = true
        } else if (gamer2.delegate.left_trigger > 0.0) {
            lift.power = gamer2.delegate.left_trigger.toDouble()
            liftRunning = true
        } else {
            lift.power = 0.0
            liftRunning = false
            lift.power = (target - lift.motor1.currentPosition) * p
        }

        // Capstone: 2
        if (gamer2.A.state) {
            transfer.capstone.position = transfer.CAP_RELEASE
        }

        // Hold lift position
        if (!liftRunning && prev) {
            target = lift.motor1.currentPosition
        }

        prev = liftRunning

        // Grabber: 2
        if (gamer2.A.pressed) {
            transfer.grabberState = Transfer.GrabberState.GRABBED
        }

        if (gamer2.Y.pressed) {
            if (transfer.grabberState == Transfer.GrabberState.GRABBED) {
                transfer.grabberState = Transfer.GrabberState.RELEASED
            } else {
                transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
            }
        }

        // Pivot: 2
        if (gamer2.X.pressed) {
            if (transfer.bar.position == transfer.BAR_DOWN) {
                transfer.pivot.position = transfer.PIVOT_LEFT
            }

            if (transfer.pivot.position == transfer.PIVOT_LEFT) {
                transfer.pivot.position = transfer.PIVOT_REGULAR
            }
        }

        if (gamer2.B.pressed) {
            if (transfer.bar.position == transfer.BAR_DOWN) {
                transfer.pivot.position = transfer.PIVOT_RIGHT
            }

            if (transfer.pivot.position == transfer.PIVOT_RIGHT) {
                transfer.pivot.position = transfer.PIVOT_REGULAR
            }
        }

        // Transfer: 2
        if (gamer2.DPAD_UP.pressed) {
            transfer.fourBarPosition = when (transfer.fourBarPosition) {
                Transfer.FourBarPosition.GRAB ->  if (transfer.grab.position == transfer.GRABBED) Transfer.FourBarPosition.PEG_ALIGN else Transfer.FourBarPosition.READY
                Transfer.FourBarPosition.READY -> Transfer.FourBarPosition.PEG_ALIGN
                Transfer.FourBarPosition.PEG_ALIGN -> Transfer.FourBarPosition.DOWN
                else -> transfer.fourBarPosition
            }
        }

        if (gamer2.DPAD_DOWN.pressed) {
            transfer.fourBarPosition = when (transfer.fourBarPosition) {
                Transfer.FourBarPosition.READY ->  Transfer.FourBarPosition.GRAB
                Transfer.FourBarPosition.PEG_ALIGN -> Transfer.FourBarPosition.READY
                Transfer.FourBarPosition.DOWN -> Transfer.FourBarPosition.PEG_ALIGN
                else -> transfer.fourBarPosition
            }
        }

        // Easy strafe: 1
        if (gamer1.DPAD_UP.pressed) {
            angleLock = drivetrain.odometer.angle
            strafing = true
        }

        if (gamer1.DPAD_UP.state) {
            drivetrain.xPower = 0.4
            drivetrain.turnPower = turn_p * (angleLock - drivetrain.odometer.angle)
        }

        if (gamer1.DPAD_DOWN.pressed) {
            angleLock = drivetrain.odometer.angle
            strafing = true
        }

        if (gamer1.DPAD_RIGHT.state) {
            drivetrain.yPower = -0.4
        }

        if (gamer1.DPAD_LEFT.state) {
            drivetrain.yPower = 0.4
        }

        if (!gamer1.DPAD_UP.state && gamer1.DPAD_UP.last) {
            strafing = false
        }

        if (!gamer1.DPAD_DOWN.state && gamer1.DPAD_DOWN.last) {
            strafing = false
        }

        if (gamer1.DPAD_DOWN.state) {
            drivetrain.xPower = -0.4
            drivetrain.turnPower = turn_p * (angleLock - drivetrain.odometer.angle)
        }

        telemetry.addData("GYRO", drivetrain.gyro.integratedZValue)
        telemetry.addData("LEFT",drivetrain.odometer.leftDeadWheel.counts)
        telemetry.addData("RIGHT",drivetrain.odometer.rightDeadWheel.counts)
        telemetry.addData("LATERAL",drivetrain.odometer.lateralDeadWheel.counts)
    }
}