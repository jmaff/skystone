package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.EnhancedGamepad
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import kotlin.math.PI

@TeleOp(name = "TeleOp")
class TeleOp: RobotOpMode() {
    lateinit var gamer1: EnhancedGamepad
    override fun start() {
        gamer1 = EnhancedGamepad(gamepad1)
        drivetrain.odometer.resetPosition(17 / 2 * 2.54, 17.75 / 2 * 2.54, PI / 2)
    }

    var turning = false
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

        if (gamer1.B.pressed) {
            if (!turning) {
                drivetrain.turnPower = 0.3
                turning = true
            } else {
                drivetrain.turnPower = 0.0
                turning = false
            }
        }

        drivetrain.xPower = gamepad1.left_stick_x.toDouble()
        drivetrain.yPower = -gamepad1.left_stick_y.toDouble()
        if (!turning) {
            drivetrain.turnPower = -gamepad1.right_stick_x.toDouble()
        }


        telemetry.addData("LEFT",drivetrain.odometer.leftDeadWheel.counts)
        telemetry.addData("RIGHT",drivetrain.odometer.rightDeadWheel.counts)
        telemetry.addData("LATERAL",drivetrain.odometer.lateralDeadWheel.counts)
    }
}