package org.firstinspires.ftc.teamcode.motion

import kotlin.math.atan2

class Line(p1: Point, p2: Point) {
    val p1: Point
    val p2: Point
    val slope: Double
        get() = (p2.y - p1.y) / (p2.x - p1.x)
    val perpSlope: Double
        get() = -1 / slope
    val angle: Double
        get() = atan2(p2.y - p1.y, p2.x - p1.x)

    init {
        var point1 = p1
        var point2 = p2
        // prevents slope from being 0 or infinity
        if (point1.x == point2.x) {
            point1 = Point(point1.x + 0.01, point1.y)
        }
        if (point1.y == point2.y) {
            point1 = Point(point1.x, point1.y + 0.01)
        }
        this.p1 = point1
        this.p2 = point2
    }

    // TODO
    fun extendedEndpoint(distance: Double): Line {
        return Line(Point(0.0,0.0), Point(0.0,0.0))
    }

    // TODO
    fun findCircleIntersections(circleCenter: Point, radius: Double): List<Point> {
        return listOf()
    }
}