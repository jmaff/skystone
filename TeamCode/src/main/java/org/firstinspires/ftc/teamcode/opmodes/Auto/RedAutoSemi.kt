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
    var missedThisStone = false
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

        telemetry.addData("State", state)

        when (state) {
            // wall to first stone
            0 -> {
                intake.state = Intake.State.IN
                transfer.fourBarPosition = Transfer.FourBarPosition.READY
                transfer.grabberState = Transfer.GrabberState.RELEASED

                var movePower = if (drivetrain.odometer.xPosition > 270.0) 0.8 else 0.3
                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                when (stonePosition) {
                    SkystoneReader.StonePosition.CENTER -> {
                        path.addWaypoint(Waypoint(295.0, 55.0, 0.8, 0.8))
                        path.addWaypoint(Waypoint(257.5, 36.0, 0.3, 0.3))
                        path.addWaypoint(Waypoint(240.0, 36.0, 0.3, 0.3))
                    }
                    SkystoneReader.StonePosition.RIGHT -> {
                        path.addWaypoint(Waypoint(295.0, 74.3, movePower, movePower))
                        path.addWaypoint(Waypoint(257.5, 56.3, movePower, movePower))
                        path.addWaypoint(Waypoint(240.0, 56.3, movePower, movePower))
                    }
                    SkystoneReader.StonePosition.LEFT -> {
                        path.addWaypoint(Waypoint(295.0, 45.0, movePower, movePower))
                        path.addWaypoint(Waypoint(257.5, 25.0, movePower, movePower))
                        path.addWaypoint(Waypoint(240.0, 17.0, movePower, movePower))
                        path.addWaypoint(Waypoint(225.0, 19.0, movePower, movePower))
                    }
                }

                // TODO: handle if a stone is NOT picked up when path is followed

                if (missedThisStone) {
                    drivetrain.yPower = 0.5
                    if (drivetrain.odometer.xPosition < 215.0) {
                        incrementState()
                    }
                } else if (drivetrain.followPath(path, toRadians(90.0))) {
                    if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                        incrementState()
                    } else {
                        missedThisStone = true
                    }
                }

                if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                    incrementState()
                }
            }
            // quarry to foundation
            1 -> {
                missedThisStone = false

                if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0 && stateTimeElapsed > 600) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                }

                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                when (stonePosition) {
                    SkystoneReader.StonePosition.CENTER -> {
                        path.addWaypoint(Waypoint(280.0, 60.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(280.0, 70.0, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.RIGHT -> {
                        path.addWaypoint(Waypoint(280.0, 85.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(280.0, 110.0, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.LEFT -> {
                        path.addWaypoint(Waypoint(280.0, 40.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(280.0, 110.0, 0.7, 0.7))
                    }
                }
                path.addWaypoint(Waypoint(280.0, 175.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(280.0, 293.0, 0.5, 0.8))
//                path.addWaypoint(Waypoint(250.0, 290.0, 0.5, 0.8))
//                path.addWaypoint(Waypoint(235.0, 290.0, 0.5, 0.8))

                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                }
            }
            // turn to foundation
            2 -> {
                transfer.grabberState = Transfer.GrabberState.GRABBED
                if (drivetrain.pointToAngle(toRadians(0.0), 0.7, toRadians(20.0))) {
                    incrementState()
                }
            }
            // back into foundation
            3 -> {
                transfer.fourBarPosition = Transfer.FourBarPosition.DOWN
                drivetrain.yPower = -0.6
                if (drivetrain.odometer.xPosition < 255.0) {
                    incrementState()
                    drivetrain.foundationDown = true
                }
            }
            // foundation to second stone
            4 -> {
                val path = Path()
                intake.state = Intake.State.IN
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(250.0, 290.0, 0.9, 0.9))
                path.addWaypoint(Waypoint(270.0, 280.0, 0.9, 0.9))
                path.addWaypoint(Waypoint(280.0, 260.0, 0.9, 0.9))
                path.addWaypoint(Waypoint(280.0, 175.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(280.0, 140.0, 0.7, 0.7))

                when (stonePosition) {
                    SkystoneReader.StonePosition.CENTER -> {
                        path.addWaypoint(Waypoint(260.0, 100.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(235.0, 90.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(230.0, 90.0, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.RIGHT -> {
                        path.addWaypoint(Waypoint(260.0, 125.3, 0.7, 0.7))
                        path.addWaypoint(Waypoint(235.0, 110.3, 0.7, 0.7))
                        path.addWaypoint(Waypoint(230.0, 110.3, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.LEFT -> {
                        path.addWaypoint(Waypoint(260.0, 80.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(235.0, 70.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(230.0, 70.0, 0.7, 0.7))
                    }
                }

//                if (drivetrain.odometer.yPosition > 250.0) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.PEG_ALIGN
//                }

                if (stateTimeElapsed < 1000) {
                    transfer.grabberState = Transfer.GrabberState.RELEASED
                }

                if (stateTimeElapsed > 1400 && drivetrain.odometer.yPosition > 200.0) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                }

                if (drivetrain.odometer.yPosition < 250.0) {
                    drivetrain.foundationDown = false
//                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                }

                if (drivetrain.odometer.yPosition < 160) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.READY
                }

                if (missedThisStone) {
                    drivetrain.yPower = 0.5
                    if (drivetrain.odometer.yPosition < 80.0) {
                        incrementState()
                    }
                    if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                        intake.state = Intake.State.OFF
                        incrementState()
                    }
                } else if (stateTimeElapsed > 600 && drivetrain.followPath(path, toRadians(90.0))) {
                    if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                        incrementState()
                        intake.state = Intake.State.OFF
                    } else {
                        missedThisStone = true
                    }
                }
            }
            // second stone to foundation
            5 -> {
                missedThisStone = false

                if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0 && !(drivetrain.odometer.yPosition > 250.0)) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                }

                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(280.0, 110.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(280.0, 175.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(280.0, 260.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(280.0, 300.0, 0.7, 0.7))

                if (drivetrain.odometer.yPosition > 115.0) {
                    intake.state = Intake.State.IN
                }

                if (drivetrain.odometer.yPosition > 200.0) {
                    transfer.grabberState = Transfer.GrabberState.GRABBED
                }

                if (drivetrain.odometer.yPosition > 250.0) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.PEG_ALIGN
                }

                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                    transfer.grabberState = Transfer.GrabberState.RELEASED
                }
            }
            // foundation to third stone seek
            6 -> {
                intake.state = Intake.State.IN
                if (stateTimeElapsed > 800 && !(drivetrain.odometer.yPosition < 150.0)) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                }

                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(280.0, 148.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(220.0, 111.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(220.0, 34.0, 0.7, 0.7))

                if (drivetrain.odometer.yPosition < 150.0) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.READY
                }

                if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                    intake.state = Intake.State.OFF
                    incrementState()
                } else if (stateTimeElapsed > 400 && drivetrain.followPath(path, toRadians(90.0))) {
                    if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                        intake.state = Intake.State.OFF
                        incrementState()
                    } else {
                        incrementState()
                        missedThisStone = true
                    }
                }
            }
            // grabbed third stone: to foundation
            7 -> {
                if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0 && !(drivetrain.odometer.yPosition > 250.0)) {
                    missedThisStone = false
                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                }

                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(280.0, 110.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(280.0, 175.0, 0.7, 0.7))

                if (!missedThisStone) {
                    path.addWaypoint(Waypoint(280.0, 260.0, 0.7, 0.7))
                    path.addWaypoint(Waypoint(280.0, 300.0, 0.7, 0.7))


                    if (drivetrain.odometer.yPosition > 200.0) {
                        transfer.grabberState = Transfer.GrabberState.GRABBED
                    }

                    if (drivetrain.odometer.yPosition > 250.0) {
                        transfer.fourBarPosition = Transfer.FourBarPosition.PEG_ALIGN
                    }
                }

                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                    transfer.grabberState = Transfer.GrabberState.RELEASED
                }
            }
            // park (if grabbed third stone)
            8 -> {
                if (!missedThisStone) {
                    if (stateTimeElapsed > 300) {
                        transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                    }

                    val path = Path()
                    path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                    path.addWaypoint(Waypoint(275.0, 178.0, 0.7, 0.7))

                    if (drivetrain.followPath(path, toRadians(90.0))) {
                        goToState(404)
                    }
                } else {
                    goToState(404)
                }
            }
            // park (if third stone missed)
            9 -> {
//                val path = Path()
//                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
//                path.addWaypoint(Waypoint(280.0, 110.0, 0.7, 0.7))
//                path.addWaypoint(Waypoint(280.0, 178.0, 0.7, 0.7))
//
//                if (drivetrain.followPath(path, toRadians(90.0))) {
//                    incrementState()
//                }
            }
            10 -> {

            }
            else -> {
                stopMovement()
            }
        }


    }
}