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

@Autonomous(name = "Blue Auto Semi")
class BlueAutoSemi: Auto() {
    lateinit var stonePosition: SkystoneReader.StonePosition
    var missedThisStone = false
    override fun init() {
        super.init()
        drivetrain.odometer.resetPosition(365.76 - 343.217, 96.282, 0.0)
        vision.enable()
        drivetrain.foundationDown = false
        transfer.grabberState = Transfer.GrabberState.RELEASED
        transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
        vision.skystoneReader.leftBound = 40
        DebugApplicationServer.clearLogPoints()
    }

    override fun init_loop() {
        super.init_loop()
        telemetry.addData("POSITION", vision.skystoneReader.stonePosition.toString())
    }

    override fun start() {
        super.start()
        drivetrain.odometer.resetPosition(365.76 - 343.217, 96.282, 0.0)
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

                var movePower = if (drivetrain.odometer.xPosition < 365.76 - 270.0) 0.8 else 0.3
                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                when (stonePosition) {
                    SkystoneReader.StonePosition.CENTER -> {
                        path.addWaypoint(Waypoint(365.76 - 295.0, 55.0, 0.8, 0.8))
                        path.addWaypoint(Waypoint(365.76 - 257.5, 36.0, 0.3, 0.3))
                        path.addWaypoint(Waypoint(365.76 - 240.0, 36.0, 0.3, 0.3))
                    }
                    SkystoneReader.StonePosition.LEFT -> {
                        path.addWaypoint(Waypoint(365.76 - 295.0, 74.3, movePower, movePower))
                        path.addWaypoint(Waypoint(365.76 - 257.5, 56.3, movePower, movePower))
                        path.addWaypoint(Waypoint(365.76 - 240.0, 56.3, movePower, movePower))
                    }
                    SkystoneReader.StonePosition.RIGHT -> {
                        path.addWaypoint(Waypoint(365.76 - 295.0, 45.0, movePower, movePower))
                        path.addWaypoint(Waypoint(365.76 - 257.5, 25.0, movePower, movePower))
                        path.addWaypoint(Waypoint(365.76 - 240.0, 17.0, movePower, movePower))
                        path.addWaypoint(Waypoint(365.76 - 225.0, 19.0, movePower, movePower))
                    }
                }

                // TODO: handle if a stone is NOT picked up when path is followed

                if (missedThisStone) {
                    drivetrain.yPower = 0.5
                    if (drivetrain.odometer.xPosition > 365.76 - 215.0) {
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
                        path.addWaypoint(Waypoint(365.76 - 280.0, 55.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 280.0, 70.0, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.LEFT -> {
                        path.addWaypoint(Waypoint(365.76 - 280.0, 55.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 280.0, 110.0, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.RIGHT -> {
                        path.addWaypoint(Waypoint(365.76 - 280.0, 40.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 280.0, 110.0, 0.7, 0.7))
                    }
                }
                path.addWaypoint(Waypoint(365.76 - 280.0, 175.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 280.0, 293.0, 0.7, 0.8))
//                path.addWaypoint(Waypoint(250.0, 290.0, 0.5, 0.8))
//                path.addWaypoint(Waypoint(235.0, 290.0, 0.5, 0.8))

                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                }
            }
            // turn to foundation
            2 -> {
                transfer.grabberState = Transfer.GrabberState.GRABBED
                if (drivetrain.pointToAngle(Math.PI, 0.7, toRadians(20.0))) {
                    incrementState()
                }
            }
            // back into foundation
            3 -> {
                transfer.fourBarPosition = Transfer.FourBarPosition.DOWN
                drivetrain.yPower = -0.6
                if (drivetrain.odometer.xPosition > 365.76 - 258.0) {
                    incrementState()
                    drivetrain.foundationDown = true
                }
            }
            // foundation to second stone
            4 -> {
                val path = Path()
                intake.state = Intake.State.IN
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(365.76 - 250.0, 290.0, 0.9, 0.9))
                path.addWaypoint(Waypoint(365.76 - 270.0, 280.0, 0.9, 0.9))
                path.addWaypoint(Waypoint(365.76 - 280.0, 260.0, 0.9, 0.9))
                path.addWaypoint(Waypoint(365.76 - 280.0, 175.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 280.0, 140.0, 0.7, 0.7))

                when (stonePosition) {
                    SkystoneReader.StonePosition.CENTER -> {
                        path.addWaypoint(Waypoint(365.76 - 260.0, 100.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 235.0, 90.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 230.0, 90.0, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.LEFT -> {
                        path.addWaypoint(Waypoint(365.76 - 260.0, 125.3, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 235.0, 110.3, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 230.0, 110.3, 0.7, 0.7))
                    }
                    SkystoneReader.StonePosition.RIGHT -> {
                        path.addWaypoint(Waypoint(365.76 - 270.0, 80.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 235.0, 70.0, 0.7, 0.7))
                        path.addWaypoint(Waypoint(365.76 - 230.0, 70.0, 0.7, 0.7))
                    }
                }

//                if (drivetrain.odometer.yPosition > 250.0) {
//                    transfer.fourBarPosition = Transfer.FourBarPosition.DOWN
//                }

                if (stateTimeElapsed > 600) {
                    transfer.grabberState = Transfer.GrabberState.RELEASED
                }

                if (stateTimeElapsed > 700 && drivetrain.odometer.yPosition > 200.0) {
                    transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                    if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                        goToState(8)
                    }

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
                path.addWaypoint(Waypoint(365.76 - 290.0, 110.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 290.0, 175.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 290.0, 260.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 290.0, 300.0, 0.7, 0.7))

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
                    if (drivetrain.stoneSensor.getDistance(DistanceUnit.CM) < 6.0) {
                        goToState(8)
                    }
                }

                val path = Path()
                path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                path.addWaypoint(Waypoint(365.76 - 285.0, 148.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 220.0, 111.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 220.0, 34.0, 0.7, 0.7))

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
                path.addWaypoint(Waypoint(365.76 - 290.0, 100.0, 0.7, 0.7))
                path.addWaypoint(Waypoint(365.76 - 290.0, 175.0, 0.7, 0.7))

                if (!missedThisStone) {
                    path.addWaypoint(Waypoint(365.76 - 290.0, 260.0, 0.7, 0.7))
                    path.addWaypoint(Waypoint(365.76 - 290.0, 300.0, 0.7, 0.7))


                    if (drivetrain.odometer.yPosition > 200.0) {
                        transfer.grabberState = Transfer.GrabberState.GRABBED
                    }

                    if (drivetrain.odometer.yPosition > 250.0) {
                        transfer.fourBarPosition = Transfer.FourBarPosition.PEG_ALIGN
                    }
                }

                if (drivetrain.followPath(path, toRadians(270.0))) {
                    incrementState()
                }
            }
            // park (if grabbed third stone)
            8 -> {
                if (!missedThisStone) {
                    if (stateTimeElapsed < 600) {
                        transfer.grabberState = Transfer.GrabberState.RELEASED
                    }

                    if (stateTimeElapsed > 600) {
                        transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
                    }

                    val path = Path()
                    path.addWaypoint(Waypoint(stateStartPosition.x, stateStartPosition.y, 0.0, 0.0, 0.0, 0.0, 0.0, true))
                    path.addWaypoint(Waypoint(365.76 - 285.0, 178.0, 0.7, 0.7))

                    if (stateTimeElapsed > 600 && drivetrain.followPath(path, toRadians(90.0))) {
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