package org.firstinspires.ftc.teamcode.hardware.subsystems

import android.os.SystemClock
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.firstinspires.ftc.teamcode.motion.*
import org.firstinspires.ftc.teamcode.telemetry.DebugApplicationServer
import org.openftc.revextensions2.ExpansionHubMotor
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kA
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kV
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.motorVelocityF
import org.firstinspires.ftc.teamcode.roadrunner.OdometryLocalizer
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil
import java.text.FieldPosition
import kotlin.math.*

@Config
class RoadrunnerDrivetrain(hardwareMap: HardwareMap) : MecanumDrive(kV, kA, kStatic, TRACK_WIDTH) {
    val topLeft: DcMotorEx = hardwareMap.get("D.TL") as DcMotorEx
    val topRight: DcMotorEx = hardwareMap.get("D.TR") as DcMotorEx
    val bottomLeft: DcMotorEx = hardwareMap.get("D.BL") as DcMotorEx
    val bottomRight: DcMotorEx = hardwareMap.get("D.BR") as DcMotorEx
    val gyro = hardwareMap.get("gyro") as ModernRoboticsI2cGyro
    val stoneSensor = hardwareMap.get("dist") as DistanceSensor

    val leftFoundation: Servo = hardwareMap.get("F.L") as Servo
    val rightFoundation: Servo = hardwareMap.get("F.R") as Servo

    val RIGHT_DOWN = 0.994 // 2488
    val RIGHT_UP = 0.6535 // 1807
    val LEFT_UP = 0.8315 // 2163
    val LEFT_DOWN = 0.5625 // 1625

    var foundationDown = false
    set(value) {
        field = value
        if (value) {
            leftFoundation.position = LEFT_DOWN
            rightFoundation.position = RIGHT_DOWN
        } else {
            leftFoundation.position = LEFT_UP
            rightFoundation.position = RIGHT_UP
        }
    }

//    @JvmField var tP = 0.0
//    @JvmField var tI = 0.0
//    @JvmField var tD = 0.0
//
//    @JvmField var hP = 0.0
//    @JvmField var hI = 0.0
//    @JvmField var hD = 0.0
//
    // Roadrunner
    val translationalPID = PIDCoefficients(tP, tI, tD)
    val headingPID = PIDCoefficients(hP, hI, hD)

    enum class Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    val motors = listOf(topLeft, topRight, bottomLeft, bottomRight)

    val dashboard: FtcDashboard
    val clock: NanoClock

    var mode: Mode

    val turnController: PIDFController
    lateinit var turnProfile: MotionProfile
    var turnStart: Double = 0.0

    val constraints: DriveConstraints
    val follower: TrajectoryFollower

    val poseHistory: MutableList<Pose2d>

    init {
        dashboard = FtcDashboard.getInstance()
        dashboard.telemetryTransmissionInterval = 25

        clock = NanoClock.system()

        mode = Mode.IDLE

        turnController = PIDFController(headingPID)

        constraints = MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)
        follower = HolonomicPIDVAFollower(translationalPID, translationalPID, headingPID, Pose2d(0.5, 0.5, toRadians(5.0)), 0.5)

        poseHistory = mutableListOf()

        localizer = OdometryLocalizer(topRight as DcMotor, bottomRight as DcMotor, bottomLeft as DcMotor)

        topLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        topRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        bottomLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        bottomRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        topLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        topRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        bottomLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        bottomRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        topLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        topRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bottomLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bottomRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        bottomRight.direction = DcMotorSimple.Direction.REVERSE
        topRight.direction = DcMotorSimple.Direction.REVERSE

        gyro.calibrate()
    }

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, false, constraints)
    }

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, constraints)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, constraints)
    }

    fun turnAsync(angle: Double) {
        val heading = poseEstimate.heading
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(heading, 0.0, 0.0, 0.0),
                MotionState(heading + angle, 0.0, 0.0, 0.0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        )
        turnStart = clock.seconds()
        mode = Mode.TURN
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun setFollowTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun getLastError(): Pose2d {
        return when (mode) {
            Mode.FOLLOW_TRAJECTORY -> follower.lastError
            Mode.TURN -> Pose2d(0.0, 0.0, turnController.lastError)
            Mode.IDLE -> Pose2d()
        }
    }

    fun update() {
        updatePoseEstimate()

        val currentPose = poseEstimate
        val (x, y, heading) = getLastError()

        poseHistory.add(currentPose)

        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        packet.put("mode", mode)

        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentPose.heading)

        packet.put("xError", x)
        packet.put("yError", y)
        packet.put("headingError", heading)

        when (mode) {
            Mode.IDLE -> {
                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("#3F51B5")
                DashboardUtil.drawRobot(fieldOverlay, currentPose)

            }
            Mode.TURN -> {
                val t = clock.seconds() - turnStart

                val targetState = turnProfile[t]

                turnController.targetPosition = targetState.x

                val correction = turnController.update(currentPose.heading)

                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                setDriveSignal(DriveSignal(Pose2d(
                        0.0, 0.0, targetOmega + correction
                ), Pose2d(
                        0.0, 0.0, targetAlpha
                )))

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
            Mode.FOLLOW_TRAJECTORY -> {
                setDriveSignal(follower.update(currentPose))

                val trajectory = follower.trajectory

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("4CAF50")
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.path)
                val t = follower.elapsedTime()
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t))

                fieldOverlay.setStroke("#3F51B5")
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory)
                DashboardUtil.drawRobot(fieldOverlay, currentPose)

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
        }

        dashboard.sendTelemetryPacket(packet)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) {
            update()
        }
    }

    fun isBusy(): Boolean {
        return mode !== Mode.IDLE
    }

    fun setMode(runMode: DcMotor.RunMode) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients {
        val coefficients = topLeft.getPIDFCoefficients(runMode)
        return PIDCoefficients(coefficients.p, coefficients.i, coefficients.d)
    }

    fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, motorVelocityF
            ))
        }
    }

    override fun getWheelPositions(): MutableList<Double> {
        val wheelPositions: MutableList<Double> = mutableListOf()
        for (motor in motors) {
            wheelPositions.add(encoderTicksToInches(motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    fun getWheelVelocities(): MutableList<Double> {
        val wheelVelocities = mutableListOf<Double>()
        for (motor in motors) {
            wheelVelocities.add(encoderTicksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    override fun setMotorPowers(v: Double, v1: Double, v2: Double, v3: Double) {
        topLeft.power = v
        bottomLeft.power = v1
        bottomRight.power = v2
        topRight.power = v3
    }

    override val rawExternalHeading: Double
        get() = gyro.integratedZValue.toDouble() //To change initializer of created properties use File | Settings | File Templates.

    companion object {
            @JvmField var tP = 0.05
            @JvmField var tI = 0.0
            @JvmField var tD = 0.0

            @JvmField var hP = 0.01
            @JvmField var hI = 0.001
            @JvmField var hD = 0.15
    }
}