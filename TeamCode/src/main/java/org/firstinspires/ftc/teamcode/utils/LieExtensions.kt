package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import org.ejml.data.DMatrix2x2
import org.ejml.data.DMatrix3x3
import org.ejml.data.DMatrixRMaj
import org.ejml.dense.fixed.CommonOps_DDF3
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

fun Twist2d.rightJacobian(): DMatrix3x3 {
    return if (angle == 0.0) {
        DMatrix3x3(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        )
    } else {
        DMatrix3x3(
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

fun Twist2d.leftJacobian(): DMatrix3x3 {
    return if (angle == 0.0) {
        DMatrix3x3(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        )
    } else {
        DMatrix3x3(
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

fun Pose2d.adjoint(): DMatrix3x3 {
    val angle = heading.log()
    val s = sin(angle)
    val c = cos(angle)
    return DMatrix3x3(
        c, -s, -position.y,
        s, c, position.x,
        0.0, 0.0, 1.0
    )
}

fun Pose2d.plusJacobians(other: Twist2d): PlusResult {
    val tau = Pose2d.exp(other)
    return PlusResult(
        times(tau),
        tau.inverse().adjoint(),
        other.rightJacobian()
    )
}

fun Pose2d.inverseJacobians(): InverseResult {
    val jSelf = adjoint()
    CommonOps_DDF3.changeSign(jSelf)
    return InverseResult(
        inverse(),
        jSelf
    )
}

fun Pose2d.timesJacobians(other: Vector2d): PointsComposeResult {
    val angle = heading.log()
    val s = sin(angle)
    val c = cos(angle)
    val jSelf = DMatrixRMaj(
        arrayOf(
            doubleArrayOf(c, -s, -position.x * s - position.y * c),
            doubleArrayOf(s, c, position.x * c - position.y * s)
        )
    )
    return PointsComposeResult(
        times(other),
        jSelf,
        DMatrix2x2(
            c, -s,
            s, c
        )
    )
}

fun Pose2d.timesJacobians(other: Pose2d): PoseComposeResult {
    return PoseComposeResult(
        times(other),
        other.inverse().adjoint()
    )
}

data class InverseResult(val pose: Pose2d, val jSelf: DMatrix3x3)
data class PlusResult(val pose: Pose2d, val jSelf: DMatrix3x3, val jTau: DMatrix3x3)
data class PointsComposeResult(val point: Vector2d, val jSelf: DMatrixRMaj, val jTau: DMatrix2x2)
data class PoseComposeResult(val pose: Pose2d, val jSelf: DMatrix3x3)