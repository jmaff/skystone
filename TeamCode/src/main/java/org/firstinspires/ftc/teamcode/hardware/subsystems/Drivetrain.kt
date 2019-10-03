package org.firstinspires.ftc.teamcode.hardware.subsystems

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.firstinspires.ftc.teamcode.motion.*
import org.firstinspires.ftc.teamcode.telemetry.DebugApplicationServer
import org.openftc.revextensions2.ExpansionHubMotor
import kotlin.math.*

class Drivetrain(hardwareMap: HardwareMap) : Subsystem() {
    val MIN_MOTOR_POWER = 0.11
    val Y_SLIP_PER_CM_PER_SEC = 0
    val X_SLIP_PER_CM_PER_SEC = 0
    val TURN_SLIP_PER_RAD_PER_SEC = 0
    val TIME_BETWEEN_UPDATES = 16

    val topLeft: OptimizedMotor = OptimizedMotor(hardwareMap.get("TL") as ExpansionHubMotor, true)
    val topRight: OptimizedMotor = OptimizedMotor(hardwareMap.get("TR") as ExpansionHubMotor, true)
    val bottomLeft: OptimizedMotor = OptimizedMotor(hardwareMap.get("BL") as ExpansionHubMotor, true)
    val bottomRight: OptimizedMotor =
        OptimizedMotor(hardwareMap.get("BR") as ExpansionHubMotor, true)

    override val motors: List<OptimizedMotor> = listOf(topLeft, topRight, bottomLeft, bottomRight)

    val odometer: Odometer = Odometer(topLeft, topRight, bottomLeft)

    var xPower = 0.0
    var yPower = 0.0
    var turnPower = 0.0

    val currentXSlip
        get() = odometer.xSpeed * X_SLIP_PER_CM_PER_SEC
    val currentYSlip
        get() = odometer.ySpeed * Y_SLIP_PER_CM_PER_SEC
    val currentTurnSlip
        get() = odometer.angularVelocity * TURN_SLIP_PER_RAD_PER_SEC

    init {
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
    }
    fun applyMotorPowers() {
        val currTime = SystemClock.uptimeMillis()
        if (currTime - lastUpdateTime < TIME_BETWEEN_UPDATES) {
            return
        }
        lastUpdateTime = currTime

        val tlRaw = yPower - turnPower + xPower
        val blRaw = yPower - turnPower - xPower
        val brRaw = -yPower - turnPower - xPower
        val trRaw = -yPower - turnPower + xPower

        val maxRaw = max(max(tlRaw, blRaw), max(brRaw, trRaw))
        val scaleDown = if (maxRaw > 1.0) 1.0 / maxRaw else 1.0

        topLeft.power = tlRaw * scaleDown
        topRight.power = trRaw * scaleDown
        bottomLeft.power = blRaw * scaleDown
        bottomRight.power = brRaw * scaleDown
    }

    fun ensureMovableMovementPowers() {
        if (abs(xPower) > abs(yPower)) {
            if (abs(xPower) > abs(turnPower)) {
                xPower = minPower(xPower, MIN_MOTOR_POWER)
            } else {
                turnPower = minPower(turnPower, MIN_MOTOR_POWER)
            }
        } else {
            if (abs(yPower) > abs(turnPower)) {
                yPower = minPower(yPower, MIN_MOTOR_POWER)
            } else {
                turnPower = minPower(turnPower, MIN_MOTOR_POWER)
            }
        }
    }

    fun goToPoint(target: Waypoint, followAngle: Double) {
        // save current position values
        val currAngle = odometer.angle
        val xCurr = odometer.xPosition
        val yCurr = odometer.yPosition

        val ySlip = (currentYSlip * sin(currAngle)) + (currentXSlip * cos(currAngle))
        val xSlip = (currentYSlip * cos(currAngle)) + (currentXSlip * sin(currAngle))

        // adjust the target point for the robot's slip
        val targetYAdj = target.y - ySlip
        val targetXAdj = target.x - xSlip

        val absoluteAngleToTarget = atan2(targetYAdj - yCurr, targetXAdj - xCurr)
        //
        val relativeAngleToTarget = wrapAngle(absoluteAngleToTarget - (currAngle - toRadians(90.0)))

        val distanceToTarget = hypot(targetXAdj - xCurr, targetYAdj - yCurr)

        // the x and y components of the vector to the adjusted point
        val relativeXToTarget = distanceToTarget * cos(relativeAngleToTarget)
        val relativeYToTarget = distanceToTarget * sin(relativeAngleToTarget)

        // normalize vector components to percentages to preserve "shape"
        var movementXPower = relativeXToTarget / (abs(relativeXToTarget) + abs(relativeYToTarget))
        var movementYPower = relativeYToTarget / (abs(relativeXToTarget) + abs(relativeYToTarget))

        // if we want to stop at this point, we will decelerate to 0% over 30 cm regardless
        if (target.stop) {
            movementXPower *= abs(relativeXToTarget) / 30.0
            movementYPower *= abs(relativeYToTarget) / 30.0
        }

        // clip the final power to be in the range we want
        xPower = Range.clip(movementXPower, -target.movementPower, target.movementPower)
        yPower = Range.clip(movementYPower, -target.movementPower, target.movementPower)

        /* Turning Stuff */

        // adjusted for what side of the robot we want pointed at the target (90 deg or pi/2 rad is
        // the front)
        val actualFollowAngle = followAngle - toRadians(90.0)
        // absolute angle to the target point, not adjusted for slip
        val rawAbsoluteAngleToTarget = hypot(target.x - xCurr, target.y - yCurr)
        // what absolute angle we should be pointing towards
        val absoluteFollowAngle = actualFollowAngle + rawAbsoluteAngleToTarget

        // how far away we are from the absolute angle we need to point to
        val relativeFollowAngle = wrapAngle(absoluteFollowAngle - currAngle)

        val adjRelativeFollowAngle = wrapAngle(relativeFollowAngle - currentTurnSlip)

        val turnSpeed = (adjRelativeFollowAngle / toRadians(30.0)) * target.turnPower

        turnPower = Range.clip(turnSpeed, -target.turnPower, target.turnPower)

        // stop turning towards the point once we get near it
        if (distanceToTarget < 10) {
            turnPower = 0.0
        }

        ensureMovableMovementPowers()

        // smooth motion during the last 6cm and 2 degrees to prevent oscillation
        xPower *= Range.clip(abs(relativeXToTarget) / 6.0, 0.0, 1.0)
        yPower *= Range.clip(abs(relativeYToTarget) / 6.0, 0.0, 1.0)
        turnPower *= Range.clip(abs(relativeFollowAngle) / toRadians(2.0), 0.0, 1.0)

        // only slow down if we are actually turning
        if (abs(turnPower) > 0.0001) {
            // slow down our movement if our current following angle is too far off
            val turnErrorMovementScaleDown = Range.clip(1.0 - abs(relativeFollowAngle /
                            target.slowDownAngle), 1.0 - target.slowDownAmount, 1.0
            )

            xPower *= turnErrorMovementScaleDown
            yPower *= turnErrorMovementScaleDown
        }
    }

    fun followPath(path: Path, followAngle: Double): Boolean {
        for (i in 0 until path.waypoints.size - 1) {
            DebugApplicationServer.sendLine(Line(path[i].toPoint(), path[i+1].toPoint()))
        }

        val extendedPath = Path(path)

        val clipped = Point(odometer.xPosition, odometer.yPosition).clippedToPath(path)
        val currentFollowIndex = clipped.index + 1

        var followPoint = extendedPath.getFollowPoint(Point(odometer.xPosition, odometer.yPosition),
            path[currentFollowIndex].followRadius)

        val lastSegment = Line(extendedPath[extendedPath.lastIndex - 1].toPoint(),
            extendedPath[extendedPath.lastIndex].toPoint())
        val extendedPoint = lastSegment.extendedEndpoint(
            extendedPath[extendedPath.lastIndex].followRadius * 1.5)
        extendedPath[extendedPath.lastIndex] = Waypoint(extendedPoint.x, extendedPoint.y,
            extendedPath[extendedPath.lastIndex])

        val pointTo = extendedPath.getFollowPoint(Point(odometer.xPosition, odometer.yPosition),
            path[currentFollowIndex].followRadius)

        val clippedDistToEnd = hypot(clipped.point.x - path[path.lastIndex].x,
            clipped.point.y - path[path.lastIndex].y)

        val actualDistToEnd = hypot(odometer.xPosition - path[path.lastIndex].x,
            odometer.yPosition - path[path.lastIndex].y)

        if (clippedDistToEnd <= followPoint.followRadius + 15 ||
            actualDistToEnd < followPoint.followRadius + 15) {
            followPoint = Waypoint(path[path.lastIndex].x, path[path.lastIndex].y, followPoint)
        }

        DebugApplicationServer.sendPoint(followPoint.toPoint())

        goToPoint(followPoint, followAngle)

        var currentFollowAngle = atan2(followPoint.y - odometer.yPosition,
            followPoint.x - odometer.xPosition)

        currentFollowAngle += wrapAngle(followAngle - PI / 2.0)

        return clippedDistToEnd < 10
    }

    fun pointToAngle(angle: Double, power: Double, decelerationAngle: Double): Boolean {
        // how far we are from the angle we want to be at
        val relativeAngle = wrapAngle(angle - odometer.angle)
        // adjusted for the slip experienced at our current angular velocity
        val adjustedRelativeAngle = wrapAngle(relativeAngle - currentTurnSlip)

        // decelerates to 0% across the angle difference specified by the client
        val turnSpeed = (adjustedRelativeAngle / decelerationAngle) * power
        // ensures the end power is within the the limit specified
        turnPower = Range.clip(turnSpeed, -power, power)

        ensureMovableMovementPowers()

        // smooths across the last 3 degrees to prevent oscillation
        turnPower *= Range.clip(abs(relativeAngle) / toRadians(3.0), 0.0, 1.0)

        return abs(relativeAngle) < toRadians(3.0)
    }
}