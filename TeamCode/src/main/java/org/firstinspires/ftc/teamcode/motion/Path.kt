package org.firstinspires.ftc.teamcode.motion

class Path(val waypoints: MutableList<Waypoint>) {
    constructor() : this(mutableListOf<Waypoint>())
    constructor(vararg waypoints: Waypoint) : this(mutableListOf(*waypoints))
    constructor(other: Path) : this(other.waypoints)

    fun addWaypoint(waypoint: Waypoint) {
        waypoints.add(waypoint)
    }

    operator fun get(index: Int): Waypoint {
        return waypoints[index]
    }

    // TODO
    fun getFollowPoint(robotLocation: Point, robotAngle: Double, followRadius: Double): Waypoint {
        return this[0]
    }

    // TODO
    fun getSegmentIndexOf(point: Point): Int {
        return 0
    }
}