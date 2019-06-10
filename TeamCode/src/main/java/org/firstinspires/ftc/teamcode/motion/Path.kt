package org.firstinspires.ftc.teamcode.motion

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

class Path(val waypoints: MutableList<Waypoint>) {
    val lastIndex
        get() = waypoints.lastIndex
    constructor() : this(mutableListOf<Waypoint>())
    constructor(vararg waypoints: Waypoint) : this(mutableListOf(*waypoints))
    constructor(other: Path) : this(mutableListOf<Waypoint>().apply { addAll(other.waypoints) })

    fun addWaypoint(waypoint: Waypoint) {
        waypoints.add(waypoint)
    }

    operator fun get(index: Int): Waypoint {
        return waypoints[index]
    }

    operator fun set(index: Int, waypoint: Waypoint) {
        waypoints[index] = waypoint
    }

    fun getFollowPoint(robotLocation: Point, followRadius: Double): Waypoint {
        val clippedToPath = robotLocation.clippedToPath(this)
        var currentFollowIndex = clippedToPath.index

        var followPoint = Waypoint(clippedToPath.point.x, clippedToPath.point.y,
            this[currentFollowIndex + 1])

        for (i in 0 until waypoints.size - 1) {
            val segment = Line(this[i].toPoint(), this[i+1].toPoint())
            val intersections = segment.findCircleIntersections(robotLocation, followRadius)

            var closestDist = Double.MAX_VALUE

            // will only be adding a follow point if we actually intersect the circle
            for (intersection in intersections) {
                val dist = hypot(intersection.x - this[waypoints.lastIndex].x,
                    intersection.y - this[waypoints.lastIndex].y)

                if (dist < closestDist) {
                    closestDist = dist
                    followPoint = Waypoint(intersection.x, intersection.y, followPoint)
                }
            }
        }
        return followPoint
    }
}