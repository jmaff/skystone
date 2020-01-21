package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.control.EnhancedGamepad
import org.firstinspires.ftc.teamcode.hardware.subsystems.Chamber
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystems.Transfer
import kotlin.math.PI

@TeleOp(name = "Blue TeleOp")
class BlueTeleOpNew: RobotOpMode() {
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
        transfer.grabberState = Transfer.GrabberState.GRABBED
        transfer.automatic = false
    }

//    var stonePossessed = false

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
//                transfer.automatic = true
//                transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
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

//        if (gamer2.LEFT_BUMPER.state) {
//            if (gamer2.delegate.right_trigger > 0.0) {
//                lift.power = 0.4
//            } else {
//                lift.power = 1.0
//            }
//            liftRunning = true
//        } else if (gamer2.RIGHT_BUMPER.state) {
//            lift.power = -1.0
//            liftRunning = true
//        } else {
//            lift.power = 0.0
//            liftRunning = false
//            lift.power = (target - lift.motor1.currentPosition) * p
//        }

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


        if (gamer1.X.state) {
            chamber.stoneOrientation = Chamber.StoneOrientation.PEGS_LEFT
        } else if (gamer1.B.state) {
            chamber.stoneOrientation = Chamber.StoneOrientation.PEGS_RIGHT
        } else {
            chamber.stoneOrientation = Chamber.StoneOrientation.NORMAL
        }

        if (gamer1.DPAD_DOWN.pressed) {
            angleLock = drivetrain.odometer.angle
            strafing = true
        }

        if (gamer1.DPAD_DOWN.state) {
            drivetrain.xPower = 0.4
            drivetrain.turnPower = turn_p * (angleLock - drivetrain.odometer.angle)
        }

        if (gamer1.DPAD_UP.pressed) {
            angleLock = drivetrain.odometer.angle
            strafing = true
        }

        if (gamer1.DPAD_LEFT.state) {
            drivetrain.yPower = -0.4
        }

        if (gamer1.DPAD_RIGHT.state) {
            drivetrain.yPower = 0.4
        }


        if (!gamer1.DPAD_UP.state && gamer1.DPAD_UP.last) {
            strafing = false
        }

        if (!gamer1.DPAD_DOWN.state && gamer1.DPAD_DOWN.last) {
            strafing = false
        }

        if (gamer1.DPAD_UP.state) {
            drivetrain.xPower = -0.4
            drivetrain.turnPower = turn_p * (angleLock - drivetrain.odometer.angle)
        }

        if (gamer2.X.state) {
            transfer.capstone.position = transfer.CAP_RELEASE
        }

//        if (gamer2.A.pressed) {
//            transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
//        }

        if (!liftRunning && prev) {
            target = lift.motor1.currentPosition
        }

        prev = liftRunning

//        if (gamer2.LEFT_JOYSTICK_PUSH.pressed) {
//            manual = !manual
//        }
            //
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

        if (!transfer.running) {
            if (gamer2.DPAD_UP.state) {
                transfer.fourBar.power = 1.0
                transfer.automatic = false
            } else if (gamer2.DPAD_DOWN.state) {
                transfer.fourBar.power = -1.0
                transfer.automatic = false
            } else {
                transfer.fourBar.power = 0.0
            }
        }

//            if (gamer2.DPAD_RIGHT.pressed) {
//                transfer.fourBarPosition = Transfer.FourBarPosition.OUT
//            }

//            if (gamer2.DPAD_LEFT.pressed) {
//                if (transfer.fourBarPosition == Transfer.FourBarPosition.GRAB) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.READY
//                } else if (transfer.fourBarPosition == Transfer.FourBarPosition.OUT) {
//                    if (transfer.grabberState != Transfer.GrabberState.PERPENDICULAR && transfer.grabberState != Transfer.GrabberState.RELEASED_PERP) {
//                        transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
//                        transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
//                    } else if (transfer.grabberState == Transfer.GrabberState.RELEASED_PERP){
//                        transfer.grabberState = Transfer.GrabberState.PERPENDICULAR
//                    } else {
//                        transfer.grabberState = Transfer.GrabberState.GRABBED
//                    }
//
//                }
//            }

//            if (gamer2.A.pressed && transfer.fourBarPosition == Transfer.FourBarPosition.OUT) {
//                transfer.grabberState = Transfer.GrabberState.PERPENDICULAR
//            }


//        chamber.stoneOrientation = when {
//            chamber.leftDist.getDistance(DistanceUnit.CM) < 5.5 && chamber.rightDist.getDistance(DistanceUnit.CM) > 7.4 && !stonePossessed-> Chamber.StoneOrientation.PEGS_LEFT
//            chamber.rightDist.getDistance(DistanceUnit.CM) < 5.5 && chamber.leftDist.getDistance(DistanceUnit.CM) > 7.4 && !stonePossessed -> Chamber.StoneOrientation.PEGS_RIGHT
//            stonePossessed -> Chamber.StoneOrientation.NORMAL
//            else -> chamber.stoneOrientation
//        }

        telemetry.addData("C LEFT", chamber.leftDist.getDistance(DistanceUnit.CM))
        telemetry.addData("C RIGHT", chamber.rightDist.getDistance(DistanceUnit.CM))
        telemetry.addData("FOUR BAR", transfer.fourBar.currentPosition)
        telemetry.addData("LEFT",drivetrain.odometer.leftDeadWheel.counts)
        telemetry.addData("RIGHT",drivetrain.odometer.rightDeadWheel.counts)
        telemetry.addData("LATERAL",drivetrain.odometer.lateralDeadWheel.counts)
    }
}