package org.firstinspires.ftc.teamcode.opmodes.Auto.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystems.Transfer
import org.firstinspires.ftc.teamcode.motion.Path
import org.firstinspires.ftc.teamcode.motion.Point
import org.firstinspires.ftc.teamcode.motion.Waypoint
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants
import org.firstinspires.ftc.teamcode.telemetry.DebugApplicationServer
import org.firstinspires.ftc.teamcode.vision.SkystoneReader

@Autonomous(name = "Red Auto RR")
class RedAutoRR: AutoRR() {
    lateinit var stonePosition: SkystoneReader.StonePosition
    var missedThisStone = false

    lateinit var wallToCenter: Trajectory
    lateinit var centerForward: Trajectory
    lateinit var centerFoundation: Trajectory

    override fun init() {
        super.init()
        vision.enable()
        drivetrain.foundationDown = false
        transfer.grabberState = Transfer.GrabberState.RELEASED
        transfer.fourBarPosition = Transfer.FourBarPosition.GRAB
        vision.skystoneReader.leftBound = 40
        DebugApplicationServer.clearLogPoints()

        wallToCenter = drivetrain.trajectoryBuilder(Pose2d(-24.0, -72 + DriveConstants.X_DIM / 2.0, toRadians(90.0)))
                .lineToConstantHeading(Vector2d(-35.5, -30.0))
                .addDisplacementMarker(MarkerCallback { drivetrain.setFollowTrajectory(centerForward) })
                .build()

        drivetrain.constraints.maxVel = 30.0
        centerForward = drivetrain.trajectoryBuilder(Pose2d(-35.5, -30.0, toRadians(90.0)))
                .lineToConstantHeading(Vector2d(-35.5, -20.0))
                .build()

        drivetrain.constraints.maxVel = 80.0
    }

    override fun init_loop() {
        super.init_loop()
        telemetry.addData("POSITION", vision.skystoneReader.stonePosition.toString())
    }

    override fun start() {
        super.start()
        drivetrain.poseEstimate = Pose2d(-24.0 - DriveConstants.X_DIM / 2.0, -72 + DriveConstants.Y_DIM / 2.0, toRadians(90.0))
        stonePosition = vision.skystoneReader.stonePosition

        drivetrain.setFollowTrajectory(
            when (stonePosition) {
                SkystoneReader.StonePosition.LEFT -> wallToCenter
                SkystoneReader.StonePosition.CENTER -> wallToCenter
                SkystoneReader.StonePosition.RIGHT -> wallToCenter
                else -> wallToCenter
            }
        )

    }

    override fun loop() {
        super.loop()

        telemetry.addData("State", state)

        when (state) {
            0 -> {
                intake.state = Intake.State.IN
                transfer.fourBarPosition = Transfer.FourBarPosition.READY
                if (!drivetrain.isBusy()) {
                    centerFoundation = drivetrain.trajectoryBuilder(drivetrain.poseEstimate)
                            .splineTo(Pose2d(-34.0, -40.0, 0.0))
                            .splineTo(Pose2d(10.0, -40.0, 0.0))
                            .splineTo(Pose2d(50.0, -40.0, 0.0))
                            .build()
                    incrementState(centerFoundation)
                }
            }
            1 -> {
                if (!drivetrain.isBusy()) {
                    incrementState()
                }
            }
        }


    }
}