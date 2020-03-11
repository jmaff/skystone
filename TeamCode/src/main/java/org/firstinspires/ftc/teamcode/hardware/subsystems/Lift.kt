package org.firstinspires.ftc.teamcode.hardware.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.devices.OptimizedMotor
import org.openftc.revextensions2.ExpansionHubMotor

class Lift(hardwareMap: HardwareMap): Subsystem {
    override var lastUpdateTime = 0L

    val motor1 = OptimizedMotor(hardwareMap.get("L.1") as ExpansionHubMotor, false)
    val motor2 = OptimizedMotor(hardwareMap.get("L.2") as ExpansionHubMotor, false)
    val motor3 = OptimizedMotor(hardwareMap.get("L.3") as ExpansionHubMotor, false)

    init {
        motor2.direction = DcMotorSimple.Direction.REVERSE
        motor1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor3.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override val motors = listOf(motor1, motor2, motor3)

    var power = 0.0
    set(value) {
        field = value
        motor1.power = value
        motor2.power = value
        motor3.power = value
    }

}