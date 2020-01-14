package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.EnhancedGamepad

@TeleOp(name = "Vision Tuner")
class TuneVision: RobotOpMode() {
    lateinit var gamer1: EnhancedGamepad
    override fun init() {
        super.init()
        vision.enable()
    }

    override fun start() {
        super.start()
        gamer1 = EnhancedGamepad(gamepad1)
    }

    override fun loop() {
        super.loop()
        if (gamer1.LEFT_BUMPER.pressed) {
            vision.skystoneReader.leftBound--
        }

        if (gamer1.RIGHT_BUMPER.pressed) {
            vision.skystoneReader.leftBound++
        }

        if (gamer1.DPAD_DOWN.pressed) {
            vision.skystoneReader.sMin--
        }

        if (gamer1.DPAD_LEFT.pressed) {
            vision.skystoneReader.vMin--
        }

        telemetry.addData("cool", vision.skystoneReader.stonePosition.toString())


    }
}