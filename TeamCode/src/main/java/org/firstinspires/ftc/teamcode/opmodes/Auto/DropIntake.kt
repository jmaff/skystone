package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystems.Transfer
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.vision.SkystoneReader

@Autonomous(name = "Deploy Intake")
class DropIntake: Auto() {
    lateinit var stonePosition: SkystoneReader.StonePosition
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(365.76 - 17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, Math.PI)
        vision.enable()
        drivetrain.foundationDown = false
        transfer.grabberState = Transfer.GrabberState.GRABBED
        vision.skystoneReader.leftBound = 0
    }

    override fun init_loop() {
        super.init_loop()
        telemetry.addData("POSITION", vision.skystoneReader.stonePosition.toString())
    }

    override fun start() {
        super.start()
//        drivetrain.odometer.resetPosition(365.76 - 17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, Math.PI)
//        stonePosition = vision.skystoneReader.stonePosition
//        transfer.automatic = true
    }

    override fun loop() {
        super.loop()
        when (state) {
            // release intake
            0 -> {
//                transfer.fourBarPosition = Transfer.FourBarPosition.READY
                intake.state = Intake.State.IN
                if (stateTimeElapsed >= 5000) {
                    incrementState()
                }
            }
            else -> {
                intake.state = Intake.State.OFF
                stopMovement()
            }
        }
    }
}