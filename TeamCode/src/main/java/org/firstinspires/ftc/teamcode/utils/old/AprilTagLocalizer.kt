package org.firstinspires.ftc.teamcode.utils.old

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.utils.old.geometry.Pose3d
import org.firstinspires.ftc.teamcode.utils.old.geometry.Rotation3d
import org.firstinspires.ftc.teamcode.utils.old.geometry.Transform3d
import org.firstinspires.ftc.teamcode.utils.old.geometry.Vector3d
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

class AprilTagLocalizer(
    var poseEstimationStrategy: PoseEstimationStrategy,
    vararg cameras: AprilTagCamera
) {
    private val cameras = cameras.asList()
    private val visionPortals: List<VisionPortal>
    private val aprilTagProcessors: List<AprilTagProcessor>
    private var referencePose: Pose3d? = null
    var numCurrentValidDetections = 0
        private set

    init {
        // add appropriate AprilTag options
        aprilTagProcessors = cameras.map { (_) ->
            AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()).build()
        }
        if (cameras.size == 1) {
            visionPortals = listOf(
                VisionPortal.Builder()
                    .setCamera(cameras[0].cameraName)
                    .addProcessor(aprilTagProcessors[0])
                    .setCameraResolution(cameras[0].resolution)
                    .setStreamFormat(cameras[0].streamFormat)
                    .enableLiveView(true)
                    .build()
            )
        } else {
            // create multi-portal thingy
            val portalIds = VisionPortal.makeMultiPortalView(
                cameras.size,
                VisionPortal.MultiPortalLayout.HORIZONTAL
            )
            visionPortals = cameras.mapIndexed { idx, cam ->
                VisionPortal.Builder()
                    .setCamera(cam.cameraName)
                    .addProcessor(aprilTagProcessors[idx])
                    .setCameraResolution(cam.resolution)
                    .setStreamFormat(cam.streamFormat)
                    .setLiveViewContainerId(portalIds[idx])
                    .build()
            }
        }
    }

    fun getReferencePose(): Pose2d? {
        return referencePose?.toPose2d()
    }

    fun setReferencePose(referencePose: Pose2d) {
        this.referencePose =
            Pose3d(
                referencePose
            )
    }

    fun getPoseEstimate(): Pose2d? {
        var poseEstimate: Pose2d? = null
        numCurrentValidDetections = 0
        when (poseEstimationStrategy) {
            PoseEstimationStrategy.CLOSEST_TO_REFERENCE_POSE -> {
                var smallestPoseDelta = Double.MAX_VALUE

                // loop over all cameras
                for (i in 0..cameras.size) {
                    val robotToCamera = cameras[i].robotToCamera
                    val aprilTagProcessor = aprilTagProcessors[i]
                    // loop over each detection in the processor
                    for (aprilTagDetection in aprilTagProcessor.detections) {
                        if (aprilTagDetection.metadata != null) {
                            // increment valid detection counter
                            numCurrentValidDetections++

                            // create a Pose3d with where the AprilTag is on the field
                            val targetPose =
                                Pose3d(
                                    Vector3d(aprilTagDetection.metadata.fieldPosition),
                                    Rotation3d(aprilTagDetection.metadata.fieldOrientation)
                                )

                            // create a Transform3d from the camera to the AprilTag
                            val ftcPose = aprilTagDetection.ftcPose
                            val cameraToTarget = Transform3d(
                                ftcPose.x, ftcPose.y, ftcPose.z,
                                Rotation3d(ftcPose.roll, ftcPose.pitch, ftcPose.yaw)
                            )

                            // find the estimated robot pose by doing two inverse transformations
                            val robotPose = targetPose
                                .transformBy(cameraToTarget.inverse())
                                .transformBy(robotToCamera.inverse())

                            // check if this is the minimum distance from the reference pose
                            val poseDelta = robotPose.position.getDistance(referencePose!!.position)
                            if (poseDelta < smallestPoseDelta) {
                                smallestPoseDelta = poseDelta
                                poseEstimate = robotPose.toPose2d()
                            }
                        } // if null, it's weird and there should probably be reporting here
                    }
                }
            }

            PoseEstimationStrategy.CLOSEST_TO_CAMERA_HEIGHT -> {
                var smallestHeightDifference = Double.MAX_VALUE

                // loop over all cameras
                for (i in 0..cameras.size) {
                    val robotToCamera = cameras[i].robotToCamera
                    val aprilTagProcessor = aprilTagProcessors[i]
                    // loop over each detection in the processor
                    // could change the estimation strategy here
                    for (aprilTagDetection in aprilTagProcessor.detections) {
                        if (aprilTagDetection.metadata != null) {
                            // increment valid detection counter
                            numCurrentValidDetections++

                            // create a Pose3d with where the AprilTag is on the field
                            val targetPose =
                                Pose3d(
                                    Vector3d(aprilTagDetection.metadata.fieldPosition),
                                    Rotation3d(aprilTagDetection.metadata.fieldOrientation)
                                )

                            // create a Transform3d from the camera to the AprilTag
                            val ftcPose = aprilTagDetection.ftcPose
                            val cameraToTarget = Transform3d(
                                ftcPose.x, ftcPose.y, ftcPose.z,
                                Rotation3d(ftcPose.roll, ftcPose.pitch, ftcPose.yaw)
                            )

                            // find the estimated robot pose by doing two inverse transformations
                            val robotPose = targetPose
                                .transformBy(cameraToTarget.inverse())
                                .transformBy(robotToCamera.inverse())

                            // check if this is the minimum height difference from the camera
                            val heightDifference = abs(
                                robotToCamera.z -
                                        targetPose
                                            .transformBy(cameraToTarget.inverse())
                                            .position
                                            .z
                            )
                            if (heightDifference < smallestHeightDifference) {
                                smallestHeightDifference = heightDifference
                                poseEstimate = robotPose.toPose2d()
                            }
                        } // if null, it's weird and there should probably be reporting here
                    }
                }
            }
        }
        return poseEstimate
    }

    // will probably try other strategies
    enum class PoseEstimationStrategy {
        CLOSEST_TO_REFERENCE_POSE,
        CLOSEST_TO_CAMERA_HEIGHT
    }
}