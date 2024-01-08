package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.abs
import kotlin.math.sign

@Config
class TwoDeadWheelLocalizer(
    hardwareMap: HardwareMap,
    private val imu: IMU,
    private val inPerTick: Double,
    override var poseEstimate: Pose2d
) : Localizer {
    @JvmField
    val par: Encoder
    @JvmField
    val perp: Encoder
    private var lastParPos: Int
    private var lastPerpPos: Int
    private var lastHeading: Rotation2d
    private var lastRawHeadingVel = 0.0
    private var headingVelOffset = 0.0
    override var poseVelocity = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        private set

    init {
        par = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "leftOuttake")))
        perp = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "leftBack")))
        lastParPos = par.getPositionAndVelocity().position
        lastPerpPos = perp.getPositionAndVelocity().position
        lastHeading = Rotation2d.exp(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))
        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS)
    }

    private val headingVelocity: Double
        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        get() {
            val rawHeadingVel =
                imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()
            if (abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
                headingVelOffset -= sign(rawHeadingVel) * 2 * Math.PI
            }
            lastRawHeadingVel = rawHeadingVel
            return headingVelOffset + rawHeadingVel
        }

    override fun update() {
        val parPosVel = par.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()
        val heading: Rotation2d =
            Rotation2d.exp(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

        val parPosDelta = parPosVel.position - lastParPos
        val perpPosDelta = perpPosVel.position - lastPerpPos
        val headingDelta = heading.minus(lastHeading)
        val headingVel = headingVelocity

        val twist = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    listOf(
                        parPosDelta - PARAMS.parYTicks * headingDelta,
                        parPosVel.velocity - PARAMS.parYTicks * headingVel
                    )
                ).times(inPerTick),
                DualNum<Time>(
                    listOf(
                        perpPosDelta - PARAMS.perpXTicks * headingDelta,
                        perpPosVel.velocity - PARAMS.perpXTicks * headingVel
                    )
                ).times(inPerTick)
            ),
            DualNum(
                listOf(
                    headingDelta,
                    headingVel
                )
            )
        )
        lastParPos = parPosVel.position
        lastPerpPos = perpPosVel.position
        lastHeading = heading

        poseEstimate = poseEstimate.plus(twist.value())
        poseVelocity = twist.velocity().value()
    }

    class Params {
        @JvmField
        var parYTicks = 0.0 // y position of the parallel encoder (in tick units)
        @JvmField
        var perpXTicks = 0.0 // x position of the perpendicular encoder (in tick units)
    }

    companion object {
        @JvmField
        var PARAMS = Params()
    }
}
