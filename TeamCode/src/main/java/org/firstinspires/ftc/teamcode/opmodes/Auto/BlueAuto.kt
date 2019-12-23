package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.vision.SkystoneReader

@Autonomous(name = "Blue Auto")
class BlueAuto: Auto() {
    lateinit var stonePosition: SkystoneReader.StonePosition
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
        vision.enable()
    }

    override fun init_loop() {
        super.init_loop()
        telemetry.addData("POSITION", vision.skystoneReader.stonePosition.toString())
    }

    override fun start() {
        super.start()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
        stonePosition = vision.skystoneReader.stonePosition
    }

    override fun loop() {
        super.loop()
        when (state) {
            // release intake
            0 -> {
                intake.state = Intake.State.OUT
                if (stateTimeElapsed >= 500) {
                    incrementState()
                }
            }
            // wall to pointed at left stone
            1 -> {
                intake.state = Intake.State.IN
                val path = Path()
                path.addWaypoint(Waypoint(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(90.0, 160.0, 0.7, 0.7))
                if (drivetrain.followPath(path, toRadians(0.0))) {
                    incrementState()
                }
            }
            2 -> {
                if (drivetrain.pointToAngle(toRadians(-42.0), 0.5, toRadians(30.0))) {
                    incrementState()
                }
            }
            3 -> {
                val path = Path()
                path.addWaypoint(Waypoint(80.0, 160.0, 0.0, 0.0))
                path.addWaypoint(Waypoint(105.5, 137.0, 0.3, 0.3))
                if (drivetrain.followPath(path, toRadians(90.0))) {
                    incrementState()
                }
            }
            4 -> {
                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
                path.addWaypoint(Waypoint(89.0, 180.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(92.0, 285.0, 0.7, 0.7))
                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                }
            }
            5 -> {
                if (drivetrain.pointToAngle(toRadians(-180.0), 0.6, toRadians(20.0))) {
                    incrementState()
                }
            }
            6 -> {
                drivetrain.yPower = -0.3
                if (stateTimeElapsed >= 600) {
                    drivetrain.foundationDown = true
                }
                if (stateTimeElapsed >= 800) {
                    incrementState()
                }
            }
            else -> {
//                intake.state = Intake.State.OFF
                stopMovement()
            }
        }
    }
}