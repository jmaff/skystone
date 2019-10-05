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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.PI

open class RobotOpMode: OpMode() {
    lateinit var drivetrain: Drivetrain
    val subsystems = mutableListOf<Subsystem>()
    lateinit var masterHub: ExpansionHubEx
    var masterBulkData: RevBulkData? = null
    var lastLoopTime: Long = 0
    val elaspedTimeThisLoop = System.currentTimeMillis() - lastLoopTime
    lateinit var grabber: Servo

    override fun init() {
        RevExtensions2.init()
        masterHub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        drivetrain = Drivetrain(hardwareMap)
        subsystems.add(drivetrain)
        grabber = hardwareMap.get(Servo::class.java, "Grabber")
        DebugApplicationServer.start()
        pollRevBulkData()
        lastLoopTime = System.currentTimeMillis()
    }

    override fun init_loop() {
        pollRevBulkData()
        DebugApplicationServer.sendRobotLocation(Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition), drivetrain.odometer.angle)
        DebugApplicationServer.markEndOfUpdate()
    }

    override fun start() {
    }

    override fun loop() {
        pollRevBulkData()
        drivetrain.odometer.updatePosition()
        drivetrain.odometer.updateSpeed()
        DebugApplicationServer.sendRobotLocation(Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition), drivetrain.odometer.angle)


        drivetrain.applyMotorPowers()

        lastLoopTime = System.currentTimeMillis()
        DebugApplicationServer.clearLogPoints()
        DebugApplicationServer.markEndOfUpdate()

        telemetry.addData("X",drivetrain.odometer.xPosition)
        telemetry.addData("Y",drivetrain.odometer.yPosition)
        telemetry.addData("ANGLE",drivetrain.odometer.angle)
    }

    fun pollRevBulkData() {
        try {
            val tempData = masterHub.bulkInputData
            if (tempData != null) {
                masterBulkData = tempData
            }
        } catch (e: Exception) {

        }
        updateMotorPositions()
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
        drivetrain.xPower = 0.0
        drivetrain.yPower = 0.0
        drivetrain.turnPower = 0.0
    }

}