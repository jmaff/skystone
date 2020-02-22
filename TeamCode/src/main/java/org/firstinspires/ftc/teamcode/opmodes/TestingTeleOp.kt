package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.control.EnhancedGamepad
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystems.Transfer
import kotlin.math.PI

@TeleOp(name = "Testing TeleOp")
class TestingTeleOp: RobotOpMode() {
    lateinit var gamer1: EnhancedGamepad
    override fun start() {
        gamer1 = EnhancedGamepad(gamepad1)
        drivetrain.odometer.resetPosition(17 / 2 * 2.54, 17.75 / 2 * 2.54, PI / 2)
        transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
    }

    var turning = false
//    var stonePossessed = false
    override fun loop() {
        super.loop()
        gamer1.update()

        if (gamer1.A.pressed && !drivetrain.foundationDown) {
            drivetrain.foundationDown = true
        } else if (gamer1.A.pressed && drivetrain.foundationDown) {
            drivetrain.foundationDown = false
        }

        if (gamer1.RIGHT_BUMPER.pressed) {
            if (intake.state == Intake.State.OFF) {
                intake.state = Intake.State.IN
            } else {
                intake.state = Intake.State.OFF
            }
        }

//        if (gamer1.B.pressed) {
//            if (!turning) {
//                drivetrain.turnPower = 0.3
//                turning = true
//            } else {
//                drivetrain.turnPower = 0.0
//                turning = false
//            }
//        }

//        if (gamer1.B.pressed) {
//            if (chamber.stoneOrientation == Chamber.StoneOrientation.NORMAL) {
//                chamber.stoneOrientation = Chamber.StoneOrientation.PEGS_LEFT
//            } else {
//                chamber.stoneOrientation = Chamber.StoneOrientation.NORMAL
//            }
//        }
//
//        if (gamer1.Y.pressed) {
//            if (chamber.stoneOrientation == Chamber.StoneOrientation.NORMAL) {
//                chamber.stoneOrientation = Chamber.StoneOrientation.PEGS_RIGHT
//            } else {
//                chamber.stoneOrientation = Chamber.StoneOrientation.NORMAL
//            }
//        }

        if (gamer1.X.pressed) {
            if (transfer.grabberState == Transfer.GrabberState.READY_FOR_STONE) {
                transfer.grabberState = Transfer.GrabberState.GRABBED
                stonePossessed = true
            } else if (transfer.grabberState == Transfer.GrabberState.GRABBED) {
                transfer.grabberState = Transfer.GrabberState.RELEASED
                stonePossessed = false
            }
        }

        drivetrain.xPower = gamepad1.left_stick_x.toDouble()
        drivetrain.yPower = -gamepad1.left_stick_y.toDouble()
        if (!turning) {
            drivetrain.turnPower = -gamepad1.right_stick_x.toDouble()
        }

//        transfer.fourBar.power = when {
//            gamer1.DPAD_UP.state -> 1.0
//            gamer1.DPAD_DOWN.state -> -1.0
//            else -> 0.0
//        }

//        if (gamer1.DPAD_UP.pressed) {
//            if (transfer.fourBarPosition == Transfer.FourBarPosition.READY) {
//                transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
//            } else if (transfer.fourBarPosition == Transfer.FourBarPosition.GRAB) {
//                transfer.fourBarPosition = Transfer.FourBarPosition.OUT
//            }
//        }
//
//        if (gamer1.DPAD_DOWN.pressed) {
//            if (transfer.fourBarPosition == Transfer.FourBarPosition.GRAB) {
//                transfer.fourBarPosition = Transfer.FourBarPosition.READY
//            } else if (transfer.fourBarPosition == Transfer.FourBarPosition.OUT) {
//                transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
//                transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
//            }
//        }


        telemetry.addData("LEFT",drivetrain.odometer.leftDeadWheel.counts)
        telemetry.addData("RIGHT",drivetrain.odometer.rightDeadWheel.counts)
        telemetry.addData("LATERAL",drivetrain.odometer.lateralDeadWheel.counts)
    }
}