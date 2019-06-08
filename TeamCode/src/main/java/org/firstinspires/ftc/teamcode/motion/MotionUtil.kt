package org.firstinspires.ftc.teamcode.motion

import kotlin.math.PI

fun wrapAngle(angle: Double): Double {
    var result = angle
    while (result < -PI) {
        result += 2 * PI
    }
    while (result > PI) {
        result -= 2 * PI
    }
    return result
}