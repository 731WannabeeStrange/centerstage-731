package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import wannabee.lie.LiePose2d
import wannabee.lie.LieTwist2d

private fun Twist2d.toLieTwist2d() = LieTwist2d(this.line.x, this.line.y, this.angle)
private fun Pose2d.toLiePose2d() = LiePose2d(this.position.x, this.position.y, this.heading.log())
private fun LiePose2d.toPose2d() = Pose2d(this.position.a1, this.position.a2, this.rotation.log())

class VisionOdometryLocalizer(
    hardwareMap: HardwareMap,
    private val inPerTick: Double,
    override var poseEstimate: Pose2d
): Localizer {
    override var poseVelocity = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        private set
    val odometryLocalizer = ThreeDeadWheelLocalizer(hardwareMap, inPerTick, poseEstimate)

    override fun update() {
        TODO("Not yet implemented")
    }
}