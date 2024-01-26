package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.ejml.data.DMatrix2x2
import org.ejml.data.DMatrix3x3
import org.ejml.equation.Equation
import org.ejml.equation.Sequence
import org.firstinspires.ftc.teamcode.utils.plusJacobians

class VisionOdometryLocalizer(
    hardwareMap: HardwareMap,
    inPerTick: Double,
    override var poseEstimate: Pose2d
): Localizer {
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
            aprilTagNoise, "R",
            Z, "Z",
            K, "K"
        )

        updateCovarianceOdometry = eq.compile("P = F*P*F' + G*W*G'")
    }

    override fun update() {
        val twist = odometryLocalizer.update()
        poseVelocity = twist.velocity().value()

        val plusResult = poseEstimate.plusJacobians(twist.value())
        poseEstimate = plusResult.pose
        J_x = plusResult.jSelf
        J_u = plusResult.jTau


    }
}