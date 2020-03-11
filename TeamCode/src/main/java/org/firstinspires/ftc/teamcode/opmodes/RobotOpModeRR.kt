package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.firstinspires.ftc.teamcode.motion.Point
import org.firstinspires.ftc.teamcode.telemetry.DebugApplicationServer
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import org.openftc.revextensions2.RevExtensions2
import java.lang.Exception
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.subsystems.*
import kotlin.math.PI

open class RobotOpModeRR: OpMode() {
    lateinit var drivetrain: RoadrunnerDrivetrain
    lateinit var intake: Intake
    lateinit var vision: Vision
    lateinit var lift: Lift
    lateinit var transfer: Transfer
    val subsystems = mutableListOf<Subsystem>()
    lateinit var masterHub: ExpansionHubEx
    lateinit var slaveHub: ExpansionHubEx
    var masterBulkData: RevBulkData? = null
    var slaveBulkData: RevBulkData? = null
    var lastLoopTime: Long = 0
    val elaspedTimeThisLoop = System.currentTimeMillis() - lastLoopTime
    lateinit var grabber: Servo

    override fun init() {
        RevExtensions2.init()
        masterHub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 3")
        slaveHub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")

        drivetrain = RoadrunnerDrivetrain(hardwareMap)

        intake = Intake(hardwareMap)
        subsystems.add(intake)

        vision = Vision(hardwareMap)
        subsystems.add(vision)

        lift = Lift(hardwareMap)
        subsystems.add(lift)

        transfer = Transfer(hardwareMap)
        subsystems.add(transfer)

        transfer.capstone.position = transfer.CAP_HOLD
        DebugApplicationServer.start()
        pollRevBulkData()
        DebugApplicationServer.clearLogPoints()
        lastLoopTime = System.currentTimeMillis()
    }

    override fun init_loop() {
        pollRevBulkData()
    }

    override fun start() {
    }

    var stonePossessed = false

    override fun loop() {
        pollRevBulkData()
        drivetrain.update()

        transfer.update()



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
        try {
            val tempData = slaveHub.bulkInputData
            if (tempData != null) {
                slaveBulkData = tempData
            }
        } catch (e: Exception) {

        }
        updateMotorPositions()
    }

    fun updateMotorPositions() {
        for (subsystem in subsystems) {
            for (motor in subsystem.motors) {
                if (motor.isMaster) {
                    masterBulkData?.let {
                        motor.currentPosition = it.getMotorCurrentPosition(motor.delegateMotor)
                    }
                } else {
                    slaveBulkData?.let {
                        motor.currentPosition = it.getMotorCurrentPosition(motor.delegateMotor)
                    }
                }
            }
        }
    }

    fun stopMovement(){
        drivetrain.setMotorPowers(0.0, 0.0, 0.0, 0.0)
    }

}