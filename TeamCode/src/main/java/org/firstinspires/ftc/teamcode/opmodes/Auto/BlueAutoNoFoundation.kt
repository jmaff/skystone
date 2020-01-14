package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystems.Transfer
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.vision.SkystoneReader

@Autonomous(name = "Blue Auto No Foundation")
class BlueAutoNoFoundation: Auto() {
    lateinit var stonePosition: SkystoneReader.StonePosition
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
        vision.enable()
        drivetrain.foundationDown = false
        transfer.grabberState = Transfer.GrabberState.GRABBED
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
//                transfer.fourBarPosition = Transfer.FourBarPosition.READY
                intake.state = Intake.State.OUT
                if (stateTimeElapsed >= 500) {
                    incrementState()
                }
            }
            // wall to pointed at left stone
            1 -> {
                intake.state = Intake.State.OFF
                transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
//                transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                val path = Path()
                path.addWaypoint(Waypoint(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0, 0.0, 0.0, 0.0, 0.0, true))
//                path.addWaypoint(Waypoint(90.0, 160.0, 0.7, 0.7))
                when (stonePosition) {
                    SkystoneReader.StonePosition.LEFT -> path.addWaypoint(Waypoint(65.0, 121.5, 0.7, 0.7))
                    SkystoneReader.StonePosition.CENTER -> path.addWaypoint(Waypoint(65.0, 102.0, 0.7, 0.7))
                    SkystoneReader.StonePosition.RIGHT -> path.addWaypoint(Waypoint(65.0, 75.0, 0.7, 0.7))
                }

                if (drivetrain.followPath(path, toRadians(90.0))) {
                    incrementState()
                }
            }
            // turn to stones
            2 -> {
                // -42
                if (drivetrain.pointToAngle(toRadians(0.0), 0.6, toRadians(10.0))) {
                    incrementState()
                }
            }
            // intake stone (drive forward)
            3 -> {
                intake.state = Intake.State.IN
                val path = Path()
//                path.addWaypoint(Waypoint(90.0, 160.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(65.0, 120.0, 0.0, 0.0))

                when (stonePosition) {
                    SkystoneReader.StonePosition.LEFT -> path.addWaypoint(Waypoint(107.5, 121.5, 0.4, 0.7))
                    SkystoneReader.StonePosition.CENTER -> path.addWaypoint(Waypoint(107.5, 102.0, 0.2, 0.7))
                    SkystoneReader.StonePosition.RIGHT -> path.addWaypoint(Waypoint(107.5, 75.0, 0.2, 0.7))
                }

                if (drivetrain.followPath(path, toRadians(90.0))) {
                    incrementState()
                }
            }
            // wait for intake
            4 -> {
                if (stateTimeElapsed > 500) {
                    incrementState()
                }
            }
            // back up
            5 -> {
                drivetrain.yPower = -0.5
                if (drivetrain.odometer.xPosition < 60) {
                    incrementState()
                }
            }
            // go to foundation and lower bar
            6 -> {
//                transfer.fourBarPosition = Transfer.FourBarPosition.READY
                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
                path.addWaypoint(Waypoint(80.0, 100.0, 1.0, 0.7))
                path.addWaypoint(Waypoint(80.0, 180.0, 1.0, 0.7))
                path.addWaypoint(Waypoint(80.0, 285.0, 1.0, 0.7))
                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                }
            }
            7 -> {
                transfer.grabberState = Transfer.GrabberState.GRABBED
                if (stateTimeElapsed >= 500) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.OUT
                    incrementState()
                }
            }
            8 -> {
                if (stateTimeElapsed > 800) {
                    transfer.grabberState = Transfer.GrabberState.RELEASED
                }
            }
            // go to second skystone
            9 -> {
                transfer.grabberState = Transfer.GrabberState.READY_FOR_STONE
                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
                path.addWaypoint(Waypoint(89.0, 180.0, 1.0, 0.7))
                when (stonePosition) {
                    SkystoneReader.StonePosition.LEFT -> path.addWaypoint(Waypoint(65.0, 58.0, 1.0, 0.7))
                    SkystoneReader.StonePosition.CENTER -> path.addWaypoint(Waypoint(65.0, 41.0, 1.0, 0.7))
                    SkystoneReader.StonePosition.RIGHT -> path.addWaypoint(Waypoint(65.0, 37.0, 1.0, 0.7))
                }
                if (drivetrain.followPath(path, toRadians(90.0))) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                    incrementState()
                }
            }
            // turn to quarry
            10 -> {
                if (stonePosition == SkystoneReader.StonePosition.RIGHT) {
                    incrementState()
                }
                if (drivetrain.pointToAngle(toRadians(0.0), 0.6, toRadians(10.0))) {
                    incrementState()
                }
            }
            // intake stone
            11 -> {
                intake.state = Intake.State.IN
                val path = Path()
//                path.addWaypoint(Waypoint(90.0, 160.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
                when (stonePosition) {
                    SkystoneReader.StonePosition.LEFT -> path.addWaypoint(Waypoint(110.0, 58.0, 0.2, 0.7))
                    SkystoneReader.StonePosition.CENTER -> path.addWaypoint(Waypoint(110.0, 41.0, 0.2, 0.7))
                    SkystoneReader.StonePosition.RIGHT -> path.addWaypoint(Waypoint(107.5, 27.0, 0.2, 0.7))
                }

                if (drivetrain.followPath(path, toRadians(90.0))) {
                    incrementState()
                }
            }
            12 -> {
                drivetrain.yPower = -0.5
                if (drivetrain.odometer.xPosition < 70) {
                    incrementState()
                }
            }
            13 -> {
//                transfer.fourBarPosition = Transfer.FourBarPosition.READY
                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
                path.addWaypoint(Waypoint(80.0, 100.0, 1.0, 0.7))
                path.addWaypoint(Waypoint(80.0, 180.0, 1.0, 0.7))
                path.addWaypoint(Waypoint(80.0, 285.0, 1.0, 0.7))
                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                }
            }
            14 -> {
                transfer.grabberState = Transfer.GrabberState.GRABBED
                if (stateTimeElapsed >= 500) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.OUT
                    incrementState()
                }
            }
            15 -> {
                if (stateTimeElapsed > 800) {
                    transfer.grabberState = Transfer.GrabberState.RELEASED
                }
            }
            else -> {
//                intake.state = Intake.State.OFF
                stopMovement()
            }
        }
    }
}