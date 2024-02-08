package org.firstinspires.ftc.teamcode.utils.localization

import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.PI

enum class AprilTagCameraMode {
    SINGLE, DOUBLE
}

class AprilTagCameras(
    hardwareMap: HardwareMap,
    cameraMode: AprilTagCameraMode
) {
    private val robotToCameraTransforms: List<Pose2d>
    private val visionPortals: List<VisionPortal>
    private val aprilTagProcessors: List<AprilTagProcessor>

    init {
        when (cameraMode) {
            AprilTagCameraMode.SINGLE -> {
                robotToCameraTransforms = listOf(
                    Pose2d(6.0, 6.0, 0.0)
                )
                aprilTagProcessors = listOf(
                    AprilTagProcessor.Builder()
                        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        .build()
                )
                visionPortals = listOf(
                    VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
                        .addProcessor(aprilTagProcessors[0])
                        .setCameraResolution(Size(800, 600))
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .enableLiveView(true)
                        .build()
                )
            }

            AprilTagCameraMode.DOUBLE -> {
                robotToCameraTransforms = listOf(
                    Pose2d(6.0, 6.0, 0.0),
                    Pose2d(-6.0, -6.0, -PI)
                )
                aprilTagProcessors = listOf(
                    AprilTagProcessor.Builder()
                        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        .build(),
                    AprilTagProcessor.Builder()
                        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        .build()
                )
                // create multi-portal thingy
                val portalIds =
                    VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL)
                visionPortals = listOf(
                    VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
                        .addProcessor(aprilTagProcessors[0])
                        .setCameraResolution(Size(800, 600))
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .setLiveViewContainerId(portalIds[0])
                        .build(),
                    VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 2"))
                        .addProcessor(aprilTagProcessors[1])
                        .setCameraResolution(Size(800, 600))
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .setLiveViewContainerId(portalIds[1])
                        .build()
                )
            }
        }
    }

    fun getCameraResults(): List<AprilTagCameraResults> {
        return aprilTagProcessors.mapIndexed { idx, processor ->
            AprilTagCameraResults(
                robotToCameraTransforms[idx],
                processor.detections
            )
        }
    }
}

data class AprilTagCameraResults(
    val robotToCameraTransform: Pose2d,
    val aprilTagDetections: ArrayList<AprilTagDetection>
)