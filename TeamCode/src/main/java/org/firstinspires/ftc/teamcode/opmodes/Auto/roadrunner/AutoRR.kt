package org.firstinspires.ftc.teamcode.opmodes.Auto.roadrunner

import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.motion.Point
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModeRR

open class AutoRR: RobotOpModeRR() {
    var state = 0
    var stateStartTime = 0L

    val stateTimeElapsed
    get() = System.currentTimeMillis() - stateStartTime

    var stateStartPosition = Point(0.0, 0.0)
    var stateStartAngle = 0.0

    fun goToState(state: Int) {
        this.state = state
        initState()
    }

    fun incrementState(){
        goToState(state + 1)
    }

    fun incrementState(trajectory: Trajectory){
        goToState(state + 1)
        drivetrain.setFollowTrajectory(trajectory)
    }

    fun initState() {
        stopMovement()
        stateStartTime = System.currentTimeMillis()
    }

    override fun start() {
        super.start()
        initState()
    }
}