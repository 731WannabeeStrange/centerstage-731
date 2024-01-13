package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d

interface Localizer {
    var poseEstimate: Pose2d
    val poseVelocity: PoseVelocity2d
    fun update()
}

class AbsoluteLocalizer(
    val incrementalLocalizer: IncrementalLocalizer,
    override var poseEstimate: Pose2d
) : Localizer {
    override var poseVelocity: PoseVelocity2d = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        private set

    override fun update() {
        val twist = incrementalLocalizer.update()
        poseEstimate = poseEstimate.plus(twist.value())
        poseVelocity = twist.velocity().value()
    }
}