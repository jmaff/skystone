package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.motion.Point
import org.firstinspires.ftc.teamcode.telemetry.DebugApplicationServer
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import org.openftc.revextensions2.RevExtensions2
import java.lang.Exception
import android.R.attr.x
import android.R.attr.y
import com.qualcomm.robotcore.eventloop.opmode.TeleOp


@TeleOp(name = "Odom Test")
class RobotOpMode: OpMode() {
    lateinit var drivetrain: Drivetrain
    val subsystems = mutableListOf<Subsystem>()
    lateinit var masterHub: ExpansionHubEx
    var masterBulkData: RevBulkData? = null
    var lastLoopTime: Long = 0
    val elaspedTimeThisLoop = System.currentTimeMillis() - lastLoopTime

    override fun init() {
        RevExtensions2.init()
        masterHub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        drivetrain = Drivetrain(hardwareMap)
        subsystems.add(drivetrain)
        DebugApplicationServer.start()
        pollRevBulkData()
        lastLoopTime = System.currentTimeMillis()
    }

    override fun init_loop() {
        pollRevBulkData()
        DebugApplicationServer.sendRobotLocation(Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition), drivetrain.odometer.angle)
        DebugApplicationServer.markEndOfUpdate()
    }

    override fun loop() {
        pollRevBulkData()
        drivetrain.odometer.updatePosition()
        DebugApplicationServer.sendRobotLocation(Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition), drivetrain.odometer.angle)

        val x = gamepad1.left_stick_x
        val y = gamepad1.left_stick_y
        val turn = -gamepad1.right_stick_x

        val r = Math.hypot(x.toDouble(), y.toDouble())
        val angle = Math.atan2(y.toDouble(), x.toDouble()) - Math.PI / 4

        drivetrain.xPower = r * Math.cos(angle)
        drivetrain.yPower = r * Math.sin(angle)
        drivetrain.turnPower = turn.toDouble()

        drivetrain.applyMotorPowers()

        lastLoopTime = System.currentTimeMillis()
    }

    fun pollRevBulkData() {
        try {
            val tempData = masterHub.bulkInputData
            if (tempData != null) {
                masterBulkData = tempData
            }
        } catch (e: Exception) {

        }
    }

    fun updateMotorPositions() {
        for (subsystem in subsystems) {
            for (motor in subsystem.motors) {
                masterBulkData?.let {
                    motor.currentPosition = it.getMotorCurrentPosition(motor.delegateMotor)
                }
            }
        }
    }
    fun stopMovement(){

    }

}