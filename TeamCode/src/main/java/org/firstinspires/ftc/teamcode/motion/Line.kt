package org.firstinspires.ftc.teamcode.motion

import java.lang.Exception
import kotlin.math.*

class Line(p1: Point, p2: Point) {
    val p1: Point
    val p2: Point
    val m: Double
        get() = (p2.y - p1.y) / (p2.x - p1.x)
    val angle: Double
        get() = atan2(p2.y - p1.y, p2.x - p1.x)

    init {
        var point1 = p1
        var point2 = p2
        // prevents slope from being 0 or infinity
        if (point1.x == point2.x) {
            point1 = Point(point1.x + 0.003, point1.y)
        }
        if (point1.y == point2.y) {
            point1 = Point(point1.x, point1.y + 0.003)
        }
        this.p1 = point1
        this.p2 = point2
    }

    // TODO
    fun extendedEndpoint(distance: Double): Point {
        val angle = atan2(p2.y - p1.y, p2.x - p1.x)
        val length = hypot(p2.x - p1.x, p2.y - p1.y)
        val extendedLength = length + distance

        val extendedX = cos(angle) * extendedLength + p1.x
        val extendedY = sin(angle) * extendedLength + p1.y
        return Point(extendedX, extendedY)
    }

    fun findCircleIntersections(circleCenter: Point, radius: Double): List<Point> {
        // redefine the first x and y to have the center of the circle be the origin
        val x1 = p1.x - circleCenter.x
        val y1 = p1.y - circleCenter.y

        // y-intercept of adjusted line
        val b = -m * x1 + y1

        // (mx+b)^2 = (r^2 - x^2)
        // quadratic: (m^2 + 1)x^2 + (2mb)x + (b^2 - r^2)
        val quadA = m.pow(2) + 1.0
//        val quadB = (2.0 * m * y1) - (2.0 * m.pow(2) * x1)
        val quadB = 2.0 * m * b
//        val quadC = ((m.pow(2) * x1.pow(2))) - (2.0 * y1 * m * x1) + y1.pow(2) - radius.pow(2)
        val quadC = b.pow(2) - radius.pow(2)

        val intersections = mutableListOf<Point>()

        try {
            // solve using the quadratic formula
            var root1X = (-quadB + sqrt(quadB.pow(2) - (4.0 * quadA * quadC))) / (2.0 * quadA)
            // use point-slope form to find the y of the root
            var root1Y = m * (root1X - x1) + y1

            // re-add the offset previously removed
            root1X += circleCenter.x
            root1Y += circleCenter.y

            val minX = min(p1.x, p2.x)
            val maxX = max(p1.x, p2.x)

            // this root is only valid if it falls within the range of the line segment we want
            if (root1X > minX && root1X < maxX) {
                intersections.add(Point(root1X, root1Y))
            }

            // repeat for the other solution
            var root2X = (-quadB - sqrt(quadB.pow(2) - (4.0 * quadA * quadC))) / (2.0 * quadA)
            var root2Y = m * (root2X - x1) + y1

            root2X += circleCenter.x
            root2Y += circleCenter.y

            if (root2X > minX && root2X < maxX) {
                intersections.add(Point(root2X, root2Y))
            }
        } catch (e: Exception) {
            // no roots were found (which we'll deal with from the caller)
        }
        return intersections
    }
}