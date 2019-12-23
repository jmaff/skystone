package org.firstinspires.ftc.teamcode.hardware.subsystems

import android.os.SystemClock
import org.firstinspires.ftc.teamcode.hardware.devices.Encoder
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.firstinspires.ftc.teamcode.motion.wrapAngle
import kotlin.math.*

class Odometer(leftDel: OptimizedMotor, rightDel: OptimizedMotor, lateralDel: OptimizedMotor) {
    val MIN_POSITION_CHANGE = 0.000000001
    val MIN_ANGLE_CHANGE = 0.000001
    val FORWARD_CM_PER_COUNT = PI * 2 * 2.54 / 4000
    val LATERAL_CM_PER_COUNT = PI * 2 * 2.54 / 4000
//    val RAD_PER_COUNT = 13.75 * 2.54 / 4000
//    val RAD_PER_COUNT = -1.099016 * 10.0.pow(-4)

    val RAD_PER_COUNT = (2 * PI) / (-30112.75 - 29890.25)

//    val RAD_PER_COUNT = -0.00029016892

    // 0.00011039

//    val p = 2.0 * PI * sqrt((6.875.pow(2) + 6.5.pow(2))/2.0)
//    val PREDICTED_LATERAL_CM_PER_RAD = -(13.75* PI / 2 * PI) * (p / 13.75 * PI)
//    val PREDICTED_LATERAL_CM_PER_RAD = -3995.744 * LATERAL_CM_PER_COUNT
    val PREDICTED_LATERAL_CM_PER_RAD = (8057.25 * LATERAL_CM_PER_COUNT) / (2 * PI)
    val TIME_BETWEEN_SPEED_UPDATES = 25

    val leftDeadWheel : Encoder = Encoder(leftDel, true)
    val rightDeadWheel: Encoder = Encoder(rightDel)
    val lateralDeadWheel: Encoder = Encoder(lateralDel)

    var lastLeftCounts = 0
        private set
    var lastRightCounts = 0
        private set
    var lastLateralCounts = 0
        private set
    var lastAngle = 0.0

    var initialLeftCounts = 0
        private set
    var initialRightCounts = 0
        private set
    var lastResetAngle = 0.0
        private set

    var xPosition = 0.0 // cm
        private set
    var yPosition = 0.0 // cm
        private set
    var angle = 0.0

    var xTraveled = 0.0 // cm
        private set
    var yTraveled = 0.0 // cm
        private set

    var xSpeed = 0.0 // cm/s
        private set
    var ySpeed = 0.0 // cm/s
        private set
    var angularVelocity = 0.0 // rad/s
        private set

    private var lastSpeedUpdateTime: Long = 0

    fun updatePosition() {
        // save current dead wheel encoder counts
        val leftCurr = leftDeadWheel.counts
        val rightCurr = rightDeadWheel.counts
        val lateralCurr = lateralDeadWheel.counts
        
        // change in encoder counts for each dead wheel
        val leftDeltaCounts = leftCurr - lastLeftCounts
        val rightDeltaCounts = rightCurr - lastRightCounts
        val lateralDeltaCounts = lateralCurr - lastLateralCounts

        // change in cm for each dead wheel
        val leftDeltaActual = leftDeltaCounts * FORWARD_CM_PER_COUNT
        val rightDeltaActual = rightDeltaCounts * FORWARD_CM_PER_COUNT
        val lateralDeltaActual = lateralDeltaCounts * LATERAL_CM_PER_COUNT

        // change in robot angle
        val angleDelta = (leftDeltaCounts - rightDeltaCounts) * RAD_PER_COUNT

        // updating our absolute angle (need to use total counts traveled)
        val totalRightCounts = rightCurr - initialRightCounts
        val totalLeftCounts = leftCurr - initialLeftCounts

        angle = wrapAngle(((totalLeftCounts - totalRightCounts) * RAD_PER_COUNT) +
                lastResetAngle)

        val lateralPrediction = angleDelta * PREDICTED_LATERAL_CM_PER_RAD

        // how much we have traveled in the x and y directions since the last update
        val relativeY: Double
        val relativeX: Double

        // if we've turned, we have to incorporate that into our movement
        if (abs(angleDelta) > 0) {
            val movementRadius = (rightDeltaActual + leftDeltaActual) / (2 * angleDelta)
            val strafeRadius = (lateralDeltaActual - lateralPrediction) / angleDelta
            relativeY = (movementRadius * sin(angleDelta)) - (strafeRadius * (1 - cos(angleDelta)))
            relativeX = (movementRadius * (1 - cos(angleDelta))) + (strafeRadius * sin(angleDelta))
        } else {
            // otherwise, the y travel is the average of the left and right wheels (both are
            // parallel to the y dimension)
            relativeY = (leftDeltaActual + rightDeltaActual) / 2.0
            // x travel is just the lateral movement corrected for predicted turn movement
            relativeX = lateralDeltaActual - lateralPrediction
        }

        // update our x and y positions using the x and y distances we've traveled
        xPosition += (cos(angle) * relativeY) + (sin(angle) * relativeX)
        yPosition += (sin(angle) * relativeY) - (cos(angle) * relativeX)

        // update our distance traveled this update to allow for speed calculation
        yTraveled += relativeY
        // don't use angle corrected x distance because idk
        xTraveled += (lateralDeltaActual - lateralPrediction)

        lastLeftCounts = leftCurr
        lastRightCounts = rightCurr
        lastLateralCounts = lateralCurr
    }

    fun updateSpeed() {
        val currTime = SystemClock.uptimeMillis()

        // don't bother calculating speed if we haven't moved
        if (abs(yTraveled) < MIN_POSITION_CHANGE && abs(xTraveled) < MIN_POSITION_CHANGE &&
            angularVelocity < MIN_ANGLE_CHANGE) {
            return
        }

        // only update speed at consistent intervals
        if (currTime - lastSpeedUpdateTime > TIME_BETWEEN_SPEED_UPDATES) {
            val elapsedSeconds = (currTime - lastSpeedUpdateTime) / 1000.0

            ySpeed = yTraveled / elapsedSeconds
            xSpeed = xTraveled / elapsedSeconds
            angularVelocity = wrapAngle(angle - lastAngle) / elapsedSeconds
            lastAngle = angle

            // reset distance traveled
            yTraveled = 0.0
            xTraveled = 0.0
            lastSpeedUpdateTime = currTime
        }
    }

    fun resetPosition(x: Double, y: Double, angle: Double) {
        xPosition = x
        yPosition = y
        this.angle = angle

        // reset our initial position and angle
        initialLeftCounts = leftDeadWheel.counts
        initialRightCounts = rightDeadWheel.counts
        lastResetAngle = angle
    }



}