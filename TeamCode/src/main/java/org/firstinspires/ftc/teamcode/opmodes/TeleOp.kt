package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.EnhancedGamepad
import kotlin.math.PI

@TeleOp(name = "TeleOp")
class TeleOp: RobotOpMode() {
    lateinit var gamer1: EnhancedGamepad
    override fun start() {
        gamer1 = EnhancedGamepad(gamepad1)
        drivetrain.odometer.resetPosition(17 / 2 * 2.54, 17.75 / 2 * 2.54, PI / 2)
    }

    override fun loop() {
        super.loop()
        gamer1.update()

        if (gamer1.A.pressed && !drivetrain.foundationDown) {
            drivetrain.setFoundationGrabberPosition(0.0)
            drivetrain.foundationDown = true
        } else if (gamer1.A.pressed && drivetrain.foundationDown) {
            drivetrain.setFoundationGrabberPosition(1.0)
            drivetrain.foundationDown = false
        }

        drivetrain.xPower = gamepad1.left_stick_x.toDouble()
        drivetrain.yPower = -gamepad1.left_stick_y.toDouble()
        drivetrain.turnPower = -gamepad1.right_stick_x.toDouble()



        telemetry.addData("LEFT",drivetrain.odometer.leftDeadWheel.counts)
        telemetry.addData("RIGHT",drivetrain.odometer.rightDeadWheel.counts)
        telemetry.addData("LATERAL",drivetrain.odometer.lateralDeadWheel.counts)
    }
}