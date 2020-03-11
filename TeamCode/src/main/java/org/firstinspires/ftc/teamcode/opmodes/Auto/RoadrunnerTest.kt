package org.firstinspires.ftc.teamcode.opmodes.Auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.subsystems.RoadrunnerDrivetrain
import org.firstinspires.ftc.teamcode.motion.toRadians
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.X_DIM
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.Y_DIM

@Autonomous(name = "Roadrunner Test")
class RoadrunnerTest: OpMode() {
    lateinit var drive: RoadrunnerDrivetrain
    lateinit var traj: Trajectory
    override fun init() {
        drive = RoadrunnerDrivetrain(hardwareMap)
        drive.poseEstimate = Pose2d(-24.0 - X_DIM / 2.0, -72 + Y_DIM / 2.0, toRadians(90.0))
        traj = drive.trajectoryBuilder(Pose2d(-24.0, -72 + X_DIM / 2.0, toRadians(90.0)))
//                .splineToConstantHeading(Pose2d(-35.5, -39.0))
//                .splineToConstantHeading(Pose2d(-35.5, -35.0))
                .splineToConstantHeading(Pose2d(-35.5, -31.0))
                .build()
    }

    override fun start() {
        super.start()
        drive.setFollowTrajectory(traj)
    }

    override fun loop() {
        drive.update()
    }

}