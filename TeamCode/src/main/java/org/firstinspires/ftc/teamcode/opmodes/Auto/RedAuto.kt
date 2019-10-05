package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Point
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.telemetry.DebugApplicationServer

@Autonomous(name = "Red Auto")
class RedAuto: RobotOpMode() {
    val grabPath = Path()
    val foundationPath = Path()
    val placePath = Path()
    val Grab2Path = Path()
    val Place2Path = Path()
    val ParkPath = Path()

    enum class States {
        GRAB1,
        FOUNDATION,
        PLACE,
        GRAB2,
        PLACE2,
        PARK
    }

    var state = States.GRAB1

    init {
        grabPath.addWaypoint(Waypoint(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0, 0.0, 0.0, 0.0, 0.0, true))
        grabPath.addWaypoint(Waypoint(80.0, 35.0, 0.5, 0.5))

        foundationPath.addWaypoint(Waypoint(80.0, 25.0, 0.0, 0.0))
        foundationPath.addWaypoint(Waypoint(90.0, 230.0, 0.5, 0.5))
        foundationPath.addWaypoint(Waypoint(150.0, 230.0, 0.5, 0.5))

        placePath.addWaypoint(Waypoint(150.0, 237.0, 0.0, 0.0))
        placePath.addWaypoint(Waypoint(30.0, 237.0, 0.5, 0.5))
    }

    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
        grabber.position = 0.5
    }

    override fun loop() {
        when (state) {
            States.GRAB1 -> {
                if (drivetrain.followPath(grabPath, toRadians(90.0))) {
                    if (drivetrain.pointToAngle(toRadians(0.0), 0.5, toRadians(30.0))) {
                        val path = Path()
                        path.addWaypoint(Waypoint(80.0, 25.0, 0.0, 0.0))
                        path.addWaypoint(Waypoint(98.0, 25.0, 0.5, 0.5))
                        if (drivetrain.followPath(path, toRadians(90.0))) {
                            grabber.position = 0.0
                            val path2 = Path()
                            path2.addWaypoint(Waypoint(98.0, 25.0, 0.0, 0.0))
                            path2.addWaypoint(Waypoint(80.0, 25.0, 0.5, 0.5))
                            if (drivetrain.followPath(path2, toRadians(-90.0))) {
                                state = States.FOUNDATION
                            }
                        }

                    }
                }
            }

            States.FOUNDATION -> {
                if (drivetrain.followPath(foundationPath, toRadians(-90.0))) {
                    if (drivetrain.pointToAngle(toRadians(-90.0), 0.7, toRadians(10.0))) {
                        state = States.PLACE
                    }
                }
            }

            States.PLACE -> {
                drivetrain.followPath(placePath, toRadians(180.0))
            }
        }

        DebugApplicationServer.logPoint(Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition))
        super.loop()
    }


}