package org.firstinspires.ftc.teamcode.opmodes.Auto

import org.firstinspires.ftc.teamcode.motion.Point
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode

open class Auto: RobotOpMode() {
    var state = 0
    var stateStartTime = 0L
    var stateStartPosition = Point(0.0, 0.0)
    var stateStartAngle = 0.0

    fun goToState(state: Int) {
        this.state = state
        initState()
    }

    fun incrementState(){
        goToState(state + 1)
    }

    fun initState() {
        stopMovement()
        stateStartTime = System.currentTimeMillis()
        stateStartPosition = Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition)
        stateStartAngle = drivetrain.odometer.angle
    }

    override fun start() {
        super.start()
        initState()
    }
}