package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import kotlin.math.PI

@Autonomous(name = "Pure Pursuit Testing")
class PurePursuitTesting: RobotOpMode() {
    val path = Path()
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24) * 2.54, 0.0)

        path.addWaypoint(Waypoint(17.75 / 2 * 2.54, (17 / 2 + 24) * 2.54, 0.0, 0.0, 0.0, 0.0, 0.0, true))
        path.addWaypoint(Waypoint(70.0, 200.0, 0.5, 0.5, 50.0, toRadians(30.0), 0.2, true))
        path.addWaypoint(Waypoint(70.0, 250.0, 0.5, 0.5, 50.0, toRadians(30.0), 0.2, true))
        path.addWaypoint(Waypoint(90.0, 270.0, 0.5, 0.5, 50.0, toRadians(30.0), 0.2, true))
    }

    override fun loop() {
        drivetrain.followPath(path, toRadians(90.0))
        super.loop()
    }
}