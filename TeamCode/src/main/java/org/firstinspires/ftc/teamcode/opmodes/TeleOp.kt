package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.PI

@TeleOp(name = "TeleOp")
class TeleOp: RobotOpMode() {
    override fun start() {
        drivetrain.odometer.resetPosition(17 / 2 * 2.54, 17.75 / 2 * 2.54, PI / 2)
    }
    override fun loop() {
        drivetrain.xPower = gamepad1.left_stick_x.toDouble()
        drivetrain.yPower = -gamepad1.left_stick_y.toDouble()
        drivetrain.turnPower = -gamepad1.right_stick_x.toDouble()
        super.loop()
        telemetry.addData("LEFT",drivetrain.odometer.leftDeadWheel.counts)
        telemetry.addData("RIGHT",drivetrain.odometer.rightDeadWheel.counts)
        telemetry.addData("LATERAL",drivetrain.odometer.lateralDeadWheel.counts)
    }
}