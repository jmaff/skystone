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
import kotlin.math.PI


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

    override fun start() {
        drivetrain.odometer.resetPosition(17 / 2 * 2.54, 17.75 / 2 * 2.54, PI / 2)
    }

    override fun loop() {
        pollRevBulkData()
        drivetrain.odometer.updatePosition()
        DebugApplicationServer.sendRobotLocation(Point(drivetrain.odometer.xPosition, drivetrain.odometer.yPosition), drivetrain.odometer.angle)


        drivetrain.xPower = gamepad1.left_stick_x.toDouble()
        drivetrain.yPower = -gamepad1.left_stick_y.toDouble()
        drivetrain.turnPower = -gamepad1.right_stick_x.toDouble()

        drivetrain.applyMotorPowers()

        telemetry.addData("LEFT",drivetrain.odometer.leftDeadWheel.counts)
        telemetry.addData("RIGHT",drivetrain.odometer.rightDeadWheel.counts)
        telemetry.addData("LATERAL",drivetrain.odometer.lateralDeadWheel.counts)

        telemetry.addData("X",drivetrain.odometer.xPosition)
        telemetry.addData("Y",drivetrain.odometer.yPosition)
        telemetry.addData("ANGLE",drivetrain.odometer.angle)

        lastLoopTime = System.currentTimeMillis()
        DebugApplicationServer.markEndOfUpdate()
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

    }

}