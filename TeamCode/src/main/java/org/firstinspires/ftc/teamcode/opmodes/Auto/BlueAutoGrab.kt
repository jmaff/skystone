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
import java.lang.Math.toDegrees

@Disabled
@Autonomous(name = "Blue Auto Grab")
class BlueAutoGrab: Auto() {
    lateinit var stonePosition: SkystoneReader.StonePosition
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
        vision.enable()
        drivetrain.foundationDown = false
        transfer.grabberState = Transfer.GrabberState.GRABBED
        vision.skystoneReader.leftBound = 40
    }

    override fun init_loop() {
        super.init_loop()
        telemetry.addData("POSITION", vision.skystoneReader.stonePosition.toString())
    }

    override fun start() {
        super.start()
//        drivetrain.odometer.resetPosition(17.75 / 2 * 2.54, (17 / 2 + 24 + (24-17)) * 2.54, 0.0)
//        stonePosition = vision.skystoneReader.stonePosition
//        transfer.automatic = true
    }

    override fun loop() {
//        super.loop()
//        when (state) {
//            // wall to first stone
//            0 -> {
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
//                when (stonePosition) {
//                    SkystoneReader.StonePosition.LEFT -> path.addWaypoint(Waypoint(78.0, 116.0, 0.8, 0.8))
//                    SkystoneReader.StonePosition.CENTER -> path.addWaypoint(Waypoint(78.0, 92.0, 0.8, 0.8))
//                    SkystoneReader.StonePosition.RIGHT -> path.addWaypoint(Waypoint(79.0, 73.0, 0.8, 0.8))
//                }
//
//                // begin movement of four bar once clear of wall
//                if (stateTimeElapsed > 400) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.OUT
//                }
//
//                // open up grabber
//                if (stateTimeElapsed > 600) {
//                    transfer.grabberState = Transfer.GrabberState.OUTSIDE_READY
//                }
//
//                // follow path backwards
//                if (drivetrain.followPath(path, toRadians(270.0))) {
//                    incrementState()
//                }
//            }
//            1 -> {
//                if (drivetrain.pointToAngle(toRadians(0.0 + 180.0), 0.6, toRadians(13.0))) {
//                    incrementState()
//                }
//            }
//            2 -> {
//                incrementState()
//            }
//            // grab stone
//            3 -> {
////                transfer.grabberState = Transfer.GrabberState.OUTSIDE_GRABBED
////                if (stateTimeElapsed >= 1000) {
////                    incrementState()
////                }
//                    drivetrain.yPower = -0.5
//                    if (drivetrain.odometer.xPosition > 365.76 - 286.0) {
//                        stopMovement()
//                        transfer.grabberState = Transfer.GrabberState.OUTSIDE_GRABBED
//                        if (stateTimeElapsed >= 1000) {
//                            incrementState()
//                        }
//                    }
//
//            }
//            // pull stone out
//            4 -> {
//                drivetrain.yPower = 0.5
//                if (drivetrain.odometer.xPosition < 60) {
//                    incrementState()
//                }
//            }
//            // go to foundation
//            5 -> {
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
//                path.addWaypoint(Waypoint(80.0, 180.0, 0.8, 0.8))
//                path.addWaypoint(Waypoint(80.0, 295.0, 0.8, 0.8))
//
//                // raise stone up to clear foundation once under bridge
////                if (drivetrain.odometer.yPosition > 229.0) {
////                    transfer.fourBarPosition = Transfer.FourBarPosition.CLEAR_FOUNDATION
////                }
//
//                if (drivetrain.followPath(path, toRadians(90.0))) {
//                    incrementState()
//                }
//            }
//            // turn to foundation
//            6 -> {
//                if (drivetrain.pointToAngle(toRadians(0.0 + 180.0), 0.8, toRadians(10.0))) {
//                    incrementState()
//                    transfer.fourBarPosition = Transfer.FourBarPosition.CLEAR_FOUNDATION
//                }
//            }
//            // back up to foundation and grab
//            7 -> {
//                drivetrain.yPower = -0.4
//                if (drivetrain.odometer.xPosition > 365.76 - 262.0) {
//                    drivetrain.foundationDown = true
//                    stopMovement()
//                    if (stateTimeElapsed >= 800) {
//                        incrementState()
//                    }
//                }
//
//            }
//            8 -> {
//                drivetrain.xPower = -0.8
//                if (drivetrain.odometer.yPosition < 290.0) {
//                    incrementState()
//                }
//            }
//            // reposition foundation
//            9 -> {
//                transfer.fourBarPosition = Transfer.FourBarPosition.OUT
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
//                path.addWaypoint(Waypoint(80.0, 255.0, 1.0, 0.8))
//                if (drivetrain.followPath(path, toRadians(90.0))) {
//                    incrementState()
//                }
//            }
//            // turn to final angle
//            10 -> {
//                if (drivetrain.pointToAngle(toRadians(-90.0), 0.8, toRadians(10.0))) {
//                    incrementState()
//                }
//            }
//            // back up with foundation and let go
//            11 -> {
//                drivetrain.yPower = -0.5
//                transfer.grabberState = Transfer.GrabberState.RELEASED
//                if (stateTimeElapsed >= 900) {
//                    drivetrain.foundationDown = false
//                    incrementState()
//                }
//            }
//            12 -> {
//                transfer.grabberState = Transfer.GrabberState.OUTSIDE_READY
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
//                path.addWaypoint(Waypoint(75.0, 180.0, 0.8, 0.8))
//                path.addWaypoint(Waypoint(75.0, 136.0, 0.8, 0.8))
//                when (stonePosition) {
//                    SkystoneReader.StonePosition.LEFT -> path.addWaypoint(Waypoint(55.0, 58.0, 0.8, 0.8))
//                    SkystoneReader.StonePosition.CENTER -> path.addWaypoint(Waypoint(55.0, 29.0, 0.8, 0.8))
//                    SkystoneReader.StonePosition.RIGHT -> path.addWaypoint(Waypoint(55.0, 45.0, 0.8, 0.8))
//                }
//                if (drivetrain.followPath(path, toRadians(90.0))) {
//                    incrementState()
//                }
//            }
//            // turn to quarry
//            13 -> {
//                when (stonePosition) {
//                    SkystoneReader.StonePosition.LEFT -> {
//                        if (drivetrain.pointToAngle(toRadians(0.0 + 180.0), 0.6, toRadians(5.0))) {
//                            incrementState()
//                        }
//                    }
//                    SkystoneReader.StonePosition.CENTER -> {
//                        if (drivetrain.pointToAngle(toRadians(180.0 - 6.0), 0.6, toRadians(5.0))) {
//                            incrementState()
//                        }
//                    }
//                    SkystoneReader.StonePosition.RIGHT -> {
//                        if (drivetrain.pointToAngle(toRadians(180.0 - 24.0), 0.6, toRadians(5.0))) {
//                            incrementState()
//                        }
//                    }
//                }
//            }
//            14 -> {
//                drivetrain.yPower = -0.5
//                if (drivetrain.odometer.xPosition > 365.76 - 294.0 && stonePosition == SkystoneReader.StonePosition.RIGHT) {
//                    stopMovement()
//                    transfer.grabberState = Transfer.GrabberState.OUTSIDE_GRABBED
//                    if (stateTimeElapsed >= 1000) {
//                        incrementState()
//                    }
//                }
//
//                if (drivetrain.odometer.xPosition > 365.76 - 292.0 && stonePosition != SkystoneReader.StonePosition.RIGHT) {
//                    stopMovement()
//                    transfer.grabberState = Transfer.GrabberState.OUTSIDE_GRABBED
//                    if (stateTimeElapsed >= 1000) {
//                        incrementState()
//                    }
//                }
//            }
//            15 -> {
//                drivetrain.yPower = 0.8
//                if (drivetrain.odometer.xPosition < 365.76 - 320.0) {
//                    incrementState()
//                }
//            }
//            16 -> {
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
//                path.addWaypoint(Waypoint(80.0, 100.0, 0.8, 0.8))
//                path.addWaypoint(Waypoint(80.0, 180.0, 0.8, 0.8))
//                path.addWaypoint(Waypoint(80.0, 285.0, 0.8, 0.8))
//                if (drivetrain.followPath(path, toRadians(270.0))) {
//                    incrementState()
//                }
//
//                if (drivetrain.odometer.yPosition > 215.0) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.CLEAR_FOUNDATION
//                }
//            }
//            17 -> {
//                transfer.fourBarPosition = Transfer.FourBarPosition.OUT
//                transfer.grabberState = Transfer.GrabberState.OUTSIDE_READY
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
//                path.addWaypoint(Waypoint(75.0, 180.0, 0.8, 0.8))
//                path.addWaypoint(Waypoint(75.0, 136.0, 0.8, 0.8))
//                when (stonePosition) {
//                    // LEFT, so grab center stone (closest)
//                    SkystoneReader.StonePosition.LEFT -> path.addWaypoint(Waypoint(55.0, 98.0, 0.8, 0.8))
//                    // center, so grab RIGHT stone (closest)
//                    SkystoneReader.StonePosition.CENTER -> path.addWaypoint(Waypoint(55.0, 114.0, 0.8, 0.8))
//                    SkystoneReader.StonePosition.RIGHT -> path.addWaypoint(Waypoint(55.0, 114.0, 0.8, 0.8))
//                }
//                if (drivetrain.followPath(path, toRadians(90.0))) {
//                    incrementState()
//                }
//            }
//            // turn to quarry
//            18 -> {
//                if (drivetrain.pointToAngle(toRadians(3.0 + 180.0), 0.6, toRadians(5.0))) {
//                    incrementState()
//                }
//            }
//            19 -> {
//                drivetrain.yPower = -0.5
//                if (drivetrain.odometer.xPosition > 365.76 - 293.0) {
//                    stopMovement()
//                    transfer.grabberState = Transfer.GrabberState.OUTSIDE_GRABBED
//                    if (stateTimeElapsed >= 1000) {
//                        incrementState()
//                    }
//                }
//            }
//            20 -> {
//                drivetrain.yPower = 0.8
//                if (drivetrain.odometer.xPosition < 365.76 - 310.0) {
//                    incrementState()
//                }
//            }
//            21-> {
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
//                path.addWaypoint(Waypoint(80.0, 100.0, 0.8, 0.8))
//                path.addWaypoint(Waypoint(80.0, 180.0, 0.8, 0.8))
//                path.addWaypoint(Waypoint(84.0, 287.0, 0.8, 0.8))
//                if (drivetrain.followPath(path, toRadians(270.0))) {
//                    transfer.grabberState = Transfer.GrabberState.OUTSIDE_READY
//                    incrementState()
//                }
//
//                if (drivetrain.odometer.yPosition > 215.0) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.CLEAR_FOUNDATION
//                }
//            }
//            22 -> {
//                    intake.state = Intake.State.OUT
//                    val path = Path()
//                    path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0))
//                    path.addWaypoint(Waypoint(80.0, 200.0, 0.8, 0.7))
//                    if (drivetrain.followPath(path, toRadians(90.0))) {
//                        incrementState()
//                    }
//            }
//        }
    }
}