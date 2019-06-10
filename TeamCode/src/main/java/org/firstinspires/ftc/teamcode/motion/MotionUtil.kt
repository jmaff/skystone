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

fun minPower(power: Double, min: Double): Double {
    return if (power in 0.0..min) {
        min
    } else if (power < 0 && power > -min) {
        -min
    } else {
        power
    }
}

fun toRadians(degrees: Double): Double {
    return degrees * PI / 180
}
