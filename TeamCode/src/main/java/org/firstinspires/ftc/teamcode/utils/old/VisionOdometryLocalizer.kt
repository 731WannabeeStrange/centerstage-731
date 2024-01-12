package org.firstinspires.ftc.teamcode.utils.old

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler
import org.firstinspires.ftc.teamcode.utils.localization.Localizer

@Config
class VisionOdometryLocalizer(
    private val odometryLocalizer: Localizer,
    private val aprilTagLocalizer: AprilTagLocalizer,
    override var poseEstimate: Pose2d,
    private val telemetryHandler: TelemetryHandler
) : Localizer {
    override var poseVelocity: PoseVelocity2d = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        private set

    override fun update() {
        odometryLocalizer.poseEstimate = poseEstimate
        odometryLocalizer.update()
        val odometryPose = odometryLocalizer.poseEstimate

        aprilTagLocalizer.setReferencePose(odometryPose)
        val aprilTagPose: Pose2d? = aprilTagLocalizer.getPoseEstimate()
        telemetryHandler.addData(
            "Odometry pose", String.format(
                "x: %.1f y: %.1f heading: %.1f",
                odometryPose.position.x, odometryPose.position.y,
                Math.toDegrees(odometryPose.heading.toDouble())
            )
        )

        if (aprilTagPose != null) {
            poseEstimate = Pose2d(
                aprilTagPose.position.times(cameraEstimateWeight)
                    .plus(odometryPose.position.times(1 - cameraEstimateWeight)),
                aprilTagPose.heading.toDouble() * cameraEstimateWeight
                        + odometryPose.heading.toDouble() * (1 - cameraEstimateWeight)
            )
            telemetryHandler.addData(
                "AprilTag pose", String.format(
                    "x: %.1f y: %.1f heading: %.1f",
                    aprilTagPose.position.x, aprilTagPose.position.y,
                    Math.toDegrees(aprilTagPose.heading.toDouble())
                )
            )
        } else {
            poseEstimate = odometryPose
            telemetryHandler.addData("AprilTag pose", "stale")
        }

        poseVelocity = odometryLocalizer.poseVelocity
    }

    companion object {
        @JvmField
        var cameraEstimateWeight = 0.02
    }
}
