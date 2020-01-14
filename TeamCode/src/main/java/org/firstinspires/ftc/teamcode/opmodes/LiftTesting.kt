package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.motors.NeveRest20Gearmotor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "LIft Testing")
class LiftTesting: RobotOpMode() {
    var lastMotor1 = 0
    var lastMotor2 = 0
    var lastMotor3 = 0
    var lastUpdateTime = 0L
    var speed1 = 0.0
    var speed2 = 0.0
    var speed3 = 0.0

    var target = 0
    var prev = false
    var liftRunning = false
    val p = 0.01

    override fun loop() {
        super.loop()
//        if (System.currentTimeMillis() - lastUpdateTime > 10) {
//            speed1 = (lift.motor1.currentPosition - lastMotor1).toDouble() / ((System.currentTimeMillis() - lastUpdateTime) / 1000.0)
//            speed1 /= 537.6
//            speed1 *= 60.0
//            speed2 = (lift.motor2.currentPosition - lastMotor2).toDouble() / ((System.currentTimeMillis() - lastUpdateTime) / 1000.0)
//            speed2 /= 537.6
//            speed2 *= 60.0
//            speed3 = (lift.motor3.currentPosition - lastMotor3).toDouble() / ((System.currentTimeMillis() - lastUpdateTime) / 1000.0)
//            speed3 /= 537.6
//            speed3 *= 60.0
//
//            lastMotor1 = lift.motor1.currentPosition
//            lastMotor2 = lift.motor2.currentPosition
//            lastMotor3 = lift.motor3.currentPosition
//            lastUpdateTime = System.currentTimeMillis()
//        }

//        telemetry.addData("Motor 1", speed1)
//        telemetry.addData("Motor 2", speed2)
//        telemetry.addData("Motor 3", speed3)
        telemetry.addData("Motor 1 Pos", lift.motor1.currentPosition)
        telemetry.addData("Motor 2 Pos", lift.motor2.currentPosition)
        telemetry.addData("Motor 3 Pos", lift.motor3.currentPosition)

        if (gamepad1.a) {
            lift.power = 1.0
            liftRunning = true
        } else if (gamepad1.b){
            lift.power = -1.0
            liftRunning = true
        } else {
            lift.power = 0.0
            liftRunning = false
            lift.power = (target - lift.motor1.currentPosition) * p
        }

        if (!liftRunning && prev) {
            target = lift.motor1.currentPosition
        }

        prev = liftRunning


    }
}