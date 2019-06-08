package org.firstinspires.ftc.teamcode.motion

class Waypoint(
        val x: Double,
        val y: Double,
        val movementPower: Double,
        val turnPower: Double,
        val followRadius: Double,
        val slowDownAngle: Double,
        val slowDownAmount: Double
) {
    constructor(other: Waypoint) : this(other.x, other.y, other.movementPower, other.turnPower,
        other.followRadius, other.slowDownAngle, other.slowDownAmount)

    fun toPoint(): Point {
        return Point(x, y)
    }
}