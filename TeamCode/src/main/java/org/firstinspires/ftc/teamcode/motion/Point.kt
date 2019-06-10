package org.firstinspires.ftc.teamcode.motion

import kotlin.math.hypot

class Point(val x: Double, val y: Double) {
    fun clippedToLine(line: Line): Point {
        val perpM = -1.0 / line.m
        // extends a line perpendicular to the line containing the point (TODO: refactor this)
        val xClipped = ((-perpM * x) + y + (line.m * line.p1.x) - line.p1.y) / (line.m - perpM)
        val yClipped = line.m * (xClipped - line.p1.x) + line.p1.y
        return Point(xClipped, yClipped)
    }

    data class IndexedPoint(val point: Point, val index: Int)
    fun clippedToPath(path: Path): IndexedPoint {
        var closestClipDistance = Double.MAX_VALUE
        var closestClipIndex = 0
        var clippedPoint: Point = Point(0.0, 0.0)

        for (i in 0 until path.waypoints.size) {
            val segment = Line(path[i].toPoint(), path[i + 1].toPoint())
            val clipped = clippedToLine(segment)
            val clipDistance = hypot(x - clipped.x, y - clipped.y)

            if (clipDistance < closestClipDistance) {
                closestClipDistance = clipDistance
                closestClipIndex = i
                clippedPoint = clipped
            }
        }
        return IndexedPoint(clippedPoint, closestClipIndex)
    }
}