package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode

@Autonomous(name = "Red Auto")
class RedAuto: Auto() {
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
        grabber.position = 0.5
    }

    override fun start() {
        super.start()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
    }

    override fun loop() {
        super.loop()
        when (state) {
            // wall to stone
            0 -> {
                val path = Path()
                path.addWaypoint(Waypoint(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(80.0, 35.0, 0.5, 0.5))
                if (drivetrain.followPath(path, toRadians(90.0))) {
                    incrementState()
                }
            }
            // ensure we are turned to face the stone
            1 -> {
                if (drivetrain.pointToAngle(0.0, 0.5, toRadians(30.0))) {
                    incrementState()
                }
            }
            // drive forward and grab stone
            2 -> {
                val path = Path()
                path.addWaypoint(Waypoint(80.0, 35.0, 0.0, 0.0))
                path.addWaypoint(Waypoint(90.0, 35.0, 0.5, 0.5))
                if (drivetrain.followPath(path, toRadians(90.0))) {
                    grabber.position = 0.1
                    incrementState()
                }
            }
            3 -> {
                if (System.currentTimeMillis() - stateStartTime >= 1000) {
                    incrementState()
                }
            }
            // back up with stone
            4 -> {
                val path = Path()
                path.addWaypoint(Waypoint(90.0, 25.0, 0.0, 0.0))
                path.addWaypoint(Waypoint(80.0, 25.0, 0.5, 0.5))
                if (drivetrain.followPath(path, toRadians(-90.0))) {
                    incrementState()
                }
            }
            // drive under bridge to foundation
            5 -> {
                val path = Path()
                path.addWaypoint(Waypoint(80.0, 25.0, 0.0, 0.0))
                path.addWaypoint(Waypoint(90.0, 230.0, 0.5, 0.5))
                if (drivetrain.followPath(path, toRadians(-90.0))) {
                    grabber.position = 0.5
                    incrementState()
                }
            }
            6 -> {
                val path = Path()
                path.addWaypoint(Waypoint(90.0, 230.0, 0.0, 0.0))
                path.addWaypoint(Waypoint(155.0, 233.0, 0.5, 0.5))
                if (drivetrain.followPath(path, toRadians(0.0))) {
                    incrementState()
                }
            }
            // ensure we are turned to line up with foundation
            7-> {
                if (drivetrain.pointToAngle(toRadians(-90.0), 0.7, toRadians(10.0))) {
                    incrementState()
                }
            }
            8 -> {
                drivetrain.yPower = -0.3
                if (System.currentTimeMillis() - stateStartTime >= 700) {
                    drivetrain.setFoundationGrabberPosition(0.0)
                }
                if (System.currentTimeMillis() - stateStartTime >= 1200) {
                    incrementState()
                }
            }
            // move foundation to build site
            9 -> {
                val path = Path()
                path.addWaypoint(Waypoint(150.0, 237.0, 0.0, 0.0))
                path.addWaypoint(Waypoint(30.0, 237.0, 0.5, 0.5))
                if (drivetrain.followPath(path, toRadians(180.0))) {
                    incrementState()
                }
            }
            // go to other skystone
            10 -> {
                val path = Path()
                path.addWaypoint(Waypoint(30.0, 237.0, 0.0, 0.0))
                path.addWaypoint(Waypoint(90.0, 190.0, 0.5, 0.5))
//                path.addWaypoint(Waypoint(98.0, 90.0, 0.5, 0.5))
                if (drivetrain.followPath(path, toRadians(90.0))) {
                    incrementState()
                }
            }
//            // ensure we are facing the skystone
//            10 -> {
//                if (drivetrain.pointToAngle(0.0, 0.5, toRadians(30.0))) {
//                    incrementState()
//                }
//            }
//            // go to foundation with other skystone
//            11 -> {
//                val path = Path()
//                path.addWaypoint(Waypoint(87.0, 90.0, 0.0, 0.0))
//                path.addWaypoint(Waypoint(30.0, 237.0, 0.5, 0.5))
//                if (drivetrain.followPath(path, toRadians(-90.0))) {
//                    incrementState()
//                }
//            }
//            // park
//            12 -> {
//                drivetrain.goToPoint(Waypoint(90.0, 190.0,0.5, 0.5), toRadians(90.0))
//            }
        }
    }
}