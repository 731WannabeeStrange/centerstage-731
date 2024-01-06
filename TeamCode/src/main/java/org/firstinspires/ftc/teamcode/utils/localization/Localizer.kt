package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d

interface Localizer {
    var poseEstimate: Pose2d
    val poseVelocity: PoseVelocity2d
    fun update()
}