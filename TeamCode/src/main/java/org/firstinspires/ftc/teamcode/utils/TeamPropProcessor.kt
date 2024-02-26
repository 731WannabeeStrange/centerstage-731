package org.firstinspires.ftc.teamcode.utils

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.imgproc.Imgproc
import kotlin.math.roundToInt

enum class Selected {
    NONE, LEFT, MIDDLE, RIGHT
}

private fun makeGraphicsRect(rect: Rect, scaleBmpPxToCanvasPx: Float): android.graphics.Rect {
    val left = (rect.x * scaleBmpPxToCanvasPx).roundToInt()
    val top = (rect.y * scaleBmpPxToCanvasPx).roundToInt()
    val right = left + (rect.width * scaleBmpPxToCanvasPx).roundToInt()
    val bottom = top + (rect.height * scaleBmpPxToCanvasPx).roundToInt()
    return android.graphics.Rect(left, top, right, bottom)
}

interface TeamPropProcessor : VisionProcessor {
    val selection: Selected
}

@Config
class UpperTeamPropProcessor : TeamPropProcessor {
    private var submat = Mat()
    private var hsvMat = Mat()
    private val rectLeft = Rect(425, 625, 40, 40)
    private val rectMiddle = Rect(875, 575, 40, 40)

    companion object {
        @JvmStatic
        val SAT_THRESHOLD = 50
    }

    override var selection = Selected.NONE
        private set

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {}
    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV)

        val satRectLeft = getAvgSaturation(hsvMat, rectLeft)
        val satRectMiddle = getAvgSaturation(hsvMat, rectMiddle)

        if (satRectLeft > SAT_THRESHOLD && satRectMiddle > SAT_THRESHOLD) {
            return if (satRectLeft > satRectMiddle) {
                Selected.LEFT
            } else {
                Selected.MIDDLE
            }
        } else if (satRectLeft > SAT_THRESHOLD) {
            return Selected.LEFT
        } else if (satRectMiddle > SAT_THRESHOLD) {
            return Selected.MIDDLE
        }

        return Selected.RIGHT
    }

    private fun getAvgSaturation(input: Mat, rect: Rect): Double {
        submat = input.submat(rect)
        val color = Core.mean(submat)
        return color.`val`[1]
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
        val selectedPaint = Paint()
        selectedPaint.color = Color.RED
        selectedPaint.style = Paint.Style.STROKE
        selectedPaint.strokeWidth = scaleCanvasDensity * 4

        val nonSelectedPaint = Paint(selectedPaint)
        nonSelectedPaint.color = Color.GREEN

        val drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx)
        val drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx)

        selection = userContext as Selected
        when (selection) {
            Selected.NONE -> {
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint)
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint)
            }

            Selected.LEFT -> {
                canvas.drawRect(drawRectangleLeft, selectedPaint)
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint)
            }

            Selected.MIDDLE -> {
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint)
                canvas.drawRect(drawRectangleMiddle, selectedPaint)
            }

            Selected.RIGHT -> {
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint)
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint)
            }
        }
    }
}

@Config
class LowerTeamPropProcessor : TeamPropProcessor {
    private var submat = Mat()
    private var hsvMat = Mat()
    private val rectRight = Rect(1225, 625, 40, 40)
    private val rectMiddle = Rect(725, 600, 40, 40)

    companion object {
        @JvmStatic
        val SAT_THRESHOLD = 50
    }

    override var selection = Selected.NONE
        private set

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {}
    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV)

        val satRectRight = getAvgSaturation(hsvMat, rectRight)
        val satRectMiddle = getAvgSaturation(hsvMat, rectMiddle)

        if (satRectRight > SAT_THRESHOLD && satRectMiddle > SAT_THRESHOLD) {
            return if (satRectRight > satRectMiddle) {
                Selected.RIGHT
            } else {
                Selected.MIDDLE
            }
        } else if (satRectRight > SAT_THRESHOLD) {
            return Selected.RIGHT
        } else if (satRectMiddle > SAT_THRESHOLD) {
            return Selected.MIDDLE
        }

        return Selected.LEFT
    }

    private fun getAvgSaturation(input: Mat, rect: Rect): Double {
        submat = input.submat(rect)
        val color = Core.mean(submat)
        return color.`val`[1]
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
        val selectedPaint = Paint()
        selectedPaint.color = Color.RED
        selectedPaint.style = Paint.Style.STROKE
        selectedPaint.strokeWidth = scaleCanvasDensity * 4

        val nonSelectedPaint = Paint(selectedPaint)
        nonSelectedPaint.color = Color.GREEN

        val drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx)
        val drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx)

        selection = userContext as Selected
        when (selection) {
            Selected.NONE -> {
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint)
            }

            Selected.RIGHT -> {
                canvas.drawRect(drawRectangleRight, selectedPaint)
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint)
            }

            Selected.MIDDLE -> {
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
                canvas.drawRect(drawRectangleMiddle, selectedPaint)
            }

            Selected.LEFT -> {
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint)
            }
        }
    }
}