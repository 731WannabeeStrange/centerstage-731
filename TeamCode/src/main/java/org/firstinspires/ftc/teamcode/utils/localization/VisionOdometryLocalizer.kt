package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.ejml.data.DMatrix2x2
import org.ejml.data.DMatrix3x3
import org.ejml.equation.Equation
import org.ejml.equation.Sequence
import wannabee.lie.LiePose2d
import wannabee.lie.LieTwist2d

private fun Twist2d.toLieTwist2d() = LieTwist2d(this.line.x, this.line.y, this.angle)
private fun Pose2d.toLiePose2d() = LiePose2d(this.position.x, this.position.y, this.heading.log())
private fun LiePose2d.toPose2d() = Pose2d(this.position.a1, this.position.a2, this.rotation.log())

class VisionOdometryLocalizer(
    hardwareMap: HardwareMap,
    private val inPerTick: Double,
    poseEstimate: Pose2d
): Localizer {
    private var liePoseEstimate = poseEstimate.toLiePose2d()
    override var poseEstimate: Pose2d
        get() = liePoseEstimate.toPose2d()
        set(value) {
            liePoseEstimate = value.toLiePose2d()
        }
    override var poseVelocity = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        private set

    private val odometryLocalizer = ThreeDeadWheelLocalizer(hardwareMap, inPerTick)
    private val odometryNoise = DMatrix3x3(
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.1
    )

    private val aprilTagCameras = AprilTagCameras(hardwareMap, AprilTagCameraMode.DOUBLE)
    private val aprilTagNoise = DMatrix2x2(
        0.1, 0.0,
        0.0, 0.1
    )

    private var P = DMatrix3x3()
    private var J_x = DMatrix3x3()
    private var J_u = DMatrix3x3()
    private val updateCovarianceOdometry: Sequence

    private var H = DMatrix3x3()
    private var Z = DMatrix3x3()
    private var K = DMatrix3x3()

    init {
        val eq = Equation()
        eq.alias(
            P, "P",
            J_x, "F",
            J_u, "G",
            odometryNoise, "W",
            H, "H",
            aprilTagNoise, "R"
                    Z, "Z",
            K, "K"
        )

        updateCovarianceOdometry = eq.compile("P = F*P*F' + G*W*G'")
    }

    override fun update() {
        val twist = odometryLocalizer.update()
        poseVelocity = twist.velocity().value()

        val lieTwist = twist.value().toLieTwist2d()
        val plusResult = liePoseEstimate.plusJacobians(lieTwist)
        liePoseEstimate = plusResult.pose
        J_x = plusResult.jSelf
        J_u = plusResult.jTau


    }
}