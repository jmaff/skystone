package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*
import org.openftc.easyopencv.OpenCvPipeline


class SkystoneReader : OpenCvPipeline() {
    val hMin = 95.0
    val hMax = 105.0
    val sMin = 160.0
    val vMin = 0.0

    var currentHSV = listOf(0.0, 0.0, 0.0)

    val leftBound = 40
    val upperBound = 75
    val stoneWidth = 90
    val stoneHeight = 40

    val percents = DoubleArray(3)

    enum class StonePosition {
        NULL,
        LEFT,
        CENTER,
        RIGHT
    }

    var stonePosition = StonePosition.NULL

    override fun processFrame(input: Mat?): Mat {
        // filters pixels that do not fall within the yellow range of a stone
        if (input != null) {
            val processingMat = Mat()
            // applies a box blur the input image
            blur(input, processingMat, Size(20.0, 20.0))
            val hsvMat = Mat()
            // convert to HSV colorspace
            cvtColor(processingMat, hsvMat, COLOR_BGR2HSV)
            val center = hsvMat[hsvMat.rows() / 2, hsvMat.cols() / 2]
            currentHSV = listOf(center[0], center[1], center[2])
            // apply HSV threshold
            Core.inRange(hsvMat, Scalar(hMin, sMin, vMin),
                    Scalar(hMax, 255.0, 255.0), hsvMat)
            val rect = Rect(input.cols() / 2, input.rows() / 2, 20, 20)
            rectangle(hsvMat, rect, Scalar(0.0, 255.0, 0.0))

//            val stoneRect = Rect(leftBound, upperBound, stoneWidth, stoneHeight)
//            rectangle(input, stoneRect, Scalar(255.0, 0.0, 0.0))
//            for (i in 0..2) {
//                val other = Rect(leftBound+stoneWidth*i, upperBound, stoneWidth, stoneHeight)
//                rectangle(input, other, Scalar(255.0, 0.0, 0.0))
//            }

            val stoneROIs = Array(3) { i ->
                Rect(leftBound + stoneWidth*i, upperBound, stoneWidth, stoneHeight)
            }

            for (i in 0 until percents.size) {
                val subMat = hsvMat.submat(stoneROIs[i])
                var white = 0
                var black = 0
                for (j in 0 until subMat.rows()) {
                    for (k in 0 until subMat.cols()) {
                        if (subMat[j, k][0] > 0.0) {
                            white++
                        } else {
                            black++
                        }
                    }
                }
                percents[i] = white.toDouble() / (black.toDouble() + white.toDouble())

                stonePosition = when (percents.indexOf(percents.min()!!)) {
                    0 -> StonePosition.LEFT
                    1 -> StonePosition.CENTER
                    2 -> StonePosition.RIGHT
                    else -> StonePosition.NULL
                }
            }



            return input
        }
        // return an empty matrix if the input is null
        stonePosition = StonePosition.NULL
        return Mat()
    }
}