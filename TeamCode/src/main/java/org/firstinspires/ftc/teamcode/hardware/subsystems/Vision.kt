package org.firstinspires.ftc.teamcode.hardware.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.SkystoneReader
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

class Vision(hardwareMap: HardwareMap): Subsystem() {
    val camera: OpenCvCamera
    val skystoneReader = SkystoneReader()

    init {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        camera = OpenCvWebcam(hardwareMap.get(WebcamName::class.java, "Camera"), cameraMonitorViewId)
        camera.openCameraDevice()
        camera.setPipeline(skystoneReader)
    }

    fun enable() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
    }

    fun disable() {
        camera.stopStreaming()
    }
}