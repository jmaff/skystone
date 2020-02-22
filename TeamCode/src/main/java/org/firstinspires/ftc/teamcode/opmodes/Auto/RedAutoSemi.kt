package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystems.Transfer
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Point
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.telemetry.DebugApplicationServer
import org.firstinspires.ftc.teamcode.vision.SkystoneReader

@Autonomous(name = "Red Auto Semi")
class RedAutoSemi: Auto() {
    lateinit var stonePosition: SkystoneReader.StonePosition
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(343.217, 96.282, Math.PI)
        vision.enable()
        drivetrain.foundationDown = false
        transfer.grabberState = Transfer.GrabberState.RELEASED
        vision.skystoneReader.leftBound = 0
        DebugApplicationServer.clearLogPoints()
    }

    override fun init_loop() {
        super.init_loop()
        telemetry.addData("POSITION", vision.skystoneReader.stonePosition.toString())
    }

    override fun start() {
        super.start()
        drivetrain.odometer.resetPosition(343.217, 96.282, Math.PI)
        stonePosition = vision.skystoneReader.stonePosition
    }

    override fun loop() {
        super.loop()
        DebugApplicationServer.logPoint(Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition))

        when (state) {
            // wall to first stone
            0 -> {
                intake.state = Intake.State.IN
                transfer.fourBarPosition = Transfer.FourBarPosition.READY
                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(280.0, 52.0, 0.8, 0.8))
                path.addWaypoint(Waypoint(240.0, 31.0, 0.8, 0.8))
                path.addWaypoint(Waypoint(230.0, 27.0, 0.4, 0.4))

                if (drivetrain.followPath(path, toRadians(90.0))) {
                    incrementState()
                }

                if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                    incrementState()
                }
            }
        }


    }
}