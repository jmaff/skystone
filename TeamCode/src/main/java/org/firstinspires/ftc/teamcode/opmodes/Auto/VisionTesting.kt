package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Vision Testing")
class VisionTesting: Auto() {
    override fun start() {
        super.start()
        vision.enable()
    }

    override fun loop() {
        super.loop()
        telemetry.addData("LEFT", vision.skystoneReader.percents[0])
        telemetry.addData("CENTER", vision.skystoneReader.percents[1])
        telemetry.addData("RIGHT", vision.skystoneReader.percents[2])

        telemetry.addData("POSITION", vision.skystoneReader.stonePosition.toString())
    }
}
