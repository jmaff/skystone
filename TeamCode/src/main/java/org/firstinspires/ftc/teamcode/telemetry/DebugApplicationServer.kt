package org.firstinspires.ftc.teamcode.telemetry

import java.lang.StringBuilder
import java.text.DecimalFormat
import org.firstinspires.ftc.teamcode.motion.Line
import org.firstinspires.ftc.teamcode.motion.Point


class DebugApplicationServer {
    val udpServer: UdpServer
    var messageBuilder = StringBuilder()
    val df = DecimalFormat()

    init {
        udpServer = UdpServer(12835)
        udpServer.active = true
        val runner = Thread(udpServer)
        runner.start()
    }

    /**
     * Sends the robot location to the debug computer
     */
    fun sendRobotLocation(point: Point, angle: Double) {
        //first send the robot location
        messageBuilder.append("ROBOT,")
        messageBuilder.append(df.format(point.x))
        messageBuilder.append(",")
        messageBuilder.append(df.format(point.y))
        messageBuilder.append(",")
        messageBuilder.append(df.format(angle))
        messageBuilder.append("%")
    }

    /**
     * Sends the location of any point you would like to send
     */
    fun sendPoint(point: Point) {
        messageBuilder.append("P,")
            .append(df.format(point.x))
            .append(",")
            .append(df.format(point.y))
            .append("%")
    }


    /**
     * This is a point you don't want to clear every update
     * @param floatPoint the point you want to send
     */
    fun logPoint(point: Point) {
        messageBuilder.append("LP,")
            .append(df.format(point.x))
            .append(",")
            .append(df.format(point.y))
            .append("%")
    }


    /**
     * Used for debugging lines
     * @param point1
     * @param point2
     */
    fun sendLine(line: Line) {
        messageBuilder.append("LINE,")
            .append(df.format(line.p1.x))
            .append(",")
            .append(df.format(line.p1.y))
            .append(",")
            .append(df.format(line.p2.x))
            .append(",")
            .append(df.format(line.p2.y))
            .append("%")
    }

    /**
     * This kills the udpServer background thread
     */
    fun stopAll() {
        udpServer.active = false
    }

    /**
     * Sends the data accumulated over the update by adding it to the udpServer
     */
    fun markEndOfUpdate() {
        messageBuilder.append("CLEAR,%")

        udpServer.addMessage(messageBuilder.toString())
        messageBuilder = StringBuilder()
    }
}