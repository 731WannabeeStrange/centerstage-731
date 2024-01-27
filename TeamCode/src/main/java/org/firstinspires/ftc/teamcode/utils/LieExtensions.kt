package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import org.ejml.data.DMatrix3x3
import org.ejml.data.DMatrixRMaj
import org.ejml.dense.fixed.CommonOps_DDF3
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

fun Twist2d.rightJacobian(matrix: DMatrix3x3) {
    if (angle == 0.0) {
        matrix.set(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        )
    } else {
        matrix.set(
            sin(angle) / angle,
            (1 - cos(angle)) / angle,
            (angle * line.x - line.y + line.y * cos(angle) - line.x * sin(angle)) / angle.pow(2),
            (cos(angle) - 1) / angle,
            sin(angle) / angle,
            (line.x + angle * line.y - line.x * cos(angle) - line.y * sin(angle)) / angle.pow(2),
            0.0,
            0.0,
            1.0
        )
    }
}

fun Twist2d.leftJacobian(matrix: DMatrix3x3) {
    if (angle == 0.0) {
        matrix.set(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        )
    } else {
        matrix.set(
            sin(angle) / angle,
            (cos(angle) - 1) / angle,
            (angle * line.x + line.y - line.y * cos(angle) - line.x * sin(angle)) / angle.pow(2),
            (1 - cos(angle)) / angle,
            sin(angle) / angle,
            (-line.x + angle * line.y + line.x * cos(angle) - line.y * sin(angle)) / angle.pow(2),
            0.0,
            0.0,
            1.0
        )
    }
}

// check this carefully later
fun Pose2d.adjoint(matrix: DMatrix3x3) {
    matrix.set(
        heading.real, -heading.imag, position.y,
        heading.imag, heading.real, -position.x,
        0.0, 0.0, 1.0
    )
}

fun Pose2d.plusJacobians(other: Twist2d, jSelf: DMatrix3x3, jTau: DMatrix3x3): Pose2d {
    val tau = Pose2d.exp(other)
    tau.inverse().adjoint(jSelf)
    other.rightJacobian(jTau)
    return times(tau)
}

fun Pose2d.inverseJacobians(jSelf: DMatrix3x3): Pose2d {
    adjoint(jSelf)
    CommonOps_DDF3.changeSign(jSelf)
    return inverse()
}

fun Pose2d.timesJacobians(other: Vector2d, jSelf: DMatrixRMaj): Vector2d {
    jSelf.set(
        arrayOf(
            doubleArrayOf(
                heading.real,
                -heading.imag,
                -position.x * heading.imag - position.y * heading.real
            ),
            doubleArrayOf(
                heading.imag,
                heading.real,
                position.x * heading.real - position.y * heading.imag
            )
        )
    )
    return times(other)
}

fun Pose2d.timesJacobians(other: Pose2d, jSelf: DMatrix3x3): Pose2d {
    other.inverse().adjoint(jSelf)
    return times(other)
}