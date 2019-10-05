package org.firstinspires.ftc.teamcode.motion

class Waypoint(
        val x: Double,
        val y: Double,
        val movementPower: Double,
        val turnPower: Double,
        val followRadius: Double = 50.0,
        val slowDownAngle: Double = toRadians(30.0),
        val slowDownAmount: Double = 0.2,
        val stop: Boolean = true
) {
    constructor(other: Waypoint) : this(other.x, other.y, other.movementPower, other.turnPower,
        other.followRadius, other.slowDownAngle, other.slowDownAmount, other.stop)

    constructor(x: Double, y: Double, other: Waypoint) : this(x, y, other.movementPower,
        other.turnPower, other.followRadius, other.slowDownAngle, other.slowDownAmount, other.stop)

    fun toPoint(): Point {
        return Point(x, y)
    }
}