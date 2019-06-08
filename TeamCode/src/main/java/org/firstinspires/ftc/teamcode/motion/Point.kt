package org.firstinspires.ftc.teamcode.motion

class Point(val x: Double, val y: Double) {
    fun clippedToLine(line: Line): Point {
        val xClipped = ((-line.perpSlope * x) + y + (line.slope * line.p1.x) - line.p1.y) /
                (line.slope - line.perpSlope)
        val yClipped = (line.slope * (xClipped - line.p1.x)) + line.p1.y
        return Point(xClipped, yClipped)
    }
}