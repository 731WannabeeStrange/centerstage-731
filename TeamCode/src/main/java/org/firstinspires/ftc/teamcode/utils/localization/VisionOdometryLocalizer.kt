package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.ejml.data.DMatrix2x2
import org.ejml.data.DMatrix3x3
import org.ejml.data.DMatrixRMaj
import org.ejml.dense.row.CommonOps_DDRM
import org.ejml.equation.Equation
import org.ejml.equation.Sequence
import org.firstinspires.ftc.teamcode.utils.inverseJacobians
import org.firstinspires.ftc.teamcode.utils.plusJacobians
import org.firstinspires.ftc.teamcode.utils.timesJacobians

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
    private var J_xi_x = DMatrix3x3()
    private var J_e_xi = DMatrixRMaj(2, 3)
    private var H = DMatrixRMaj(2, 3)
    private var Z = DMatrix2x2()
    private var K = DMatrixRMaj(3, 2)

    private val updateCovarianceOdometry: Sequence
    private val updateH: Sequence
    private val updateCameraCovariance: Sequence
    private val updateKalmanGain: Sequence
    private val updateCovarianceMeasurement: Sequence

    init {
        val eq = Equation()
        eq.alias(
            P, "P",
            J_x, "F",
            J_u, "G",
            J_xi_x, "I",
            J_e_xi, "J",
            odometryNoise, "W",
            H, "H",
            aprilTagNoise, "R",
            Z, "Z",
            K, "K"
        )

        updateCovarianceOdometry = eq.compile("P = F*P*F' + G*W*G'")
        updateH = eq.compile("H = J*I")
        updateCameraCovariance = eq.compile("Z = H*P*H' + R")
        updateKalmanGain = eq.compile("K = P*H'*inv(Z)")
        updateCovarianceMeasurement = eq.compile("P = P - K*Z*K'")
    }

    override fun update() {
        // get odometry signal
        val twist = odometryLocalizer.update()
        poseVelocity = twist.velocity().value()

        // move
        poseEstimate = poseEstimate.plusJacobians(twist.value(), J_x, J_u)
        updateCovarianceOdometry.perform() // these steps alone should do odo localization right

        // correct using each landmark
        for (result in aprilTagCameras.getCameraResults()) {
            for (detection in result.aprilTagDetections) {
                // landmark
                val b = Vector2d(
                    detection.metadata.fieldPosition.get(0).toDouble(),
                    detection.metadata.fieldPosition.get(1).toDouble()
                )

                // measurement
                val y = Vector2d(
                    detection.ftcPose.x,
                    detection.ftcPose.y
                )

                // expectation
                val e = (poseEstimate * result.robotToCameraTransform).inverseJacobians(J_xi_x)
                    .timesJacobians(b, J_e_xi)
                updateH.perform()

                // innovation
                val z = y - e
                updateCameraCovariance.perform()

                // Kalman gain
                updateKalmanGain.perform()

                // correction
                val dx = DMatrixRMaj()
                CommonOps_DDRM.mult(K, DMatrixRMaj(doubleArrayOf(z.x, z.y)), dx)

                // update
                poseEstimate += Twist2d(Vector2d(dx.get(0), dx.get(1)), dx.get(2))
                updateCovarianceMeasurement.perform()
            }
        }
    }
}