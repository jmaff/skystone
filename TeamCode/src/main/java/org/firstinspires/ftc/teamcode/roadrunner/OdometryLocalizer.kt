package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor

class OdometryLocalizer(val leftEncoder: DcMotor, val rightEncoder: DcMotor, val lateralEncoder: DcMotor) : ThreeTrackingWheelLocalizer(listOf(
        Pose2d(0.625, LATERAL_DISTANCE / 2, 0.0), // left
        Pose2d(0.625, -LATERAL_DISTANCE / 2, 0.0), // right
        Pose2d(-1.375, -LATERAL_DISTANCE / 2, Math.toRadians(90.0)) // front,
)) {

    override fun getWheelPositions(): List<Double> {
        return listOf(
                encoderTicksToInches(leftEncoder.currentPosition),
                encoderTicksToInches(rightEncoder.currentPosition),
                encoderTicksToInches(lateralEncoder.currentPosition)
        )
    }

    companion object {
        var TICKS_PER_REV = -4000.0
        var WHEEL_RADIUS = 1.0 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed

        var LATERAL_DISTANCE = 14.75 // in; distance between the left and right wheels
        var FORWARD_OFFSET = 0.625 // in; offset of the lateral wheel

        fun encoderTicksToInches(ticks: Int): Double {
            return WHEEL_RADIUS * 2.0 * Math.PI * GEAR_RATIO * ticks.toDouble() / TICKS_PER_REV
        }
    }
}