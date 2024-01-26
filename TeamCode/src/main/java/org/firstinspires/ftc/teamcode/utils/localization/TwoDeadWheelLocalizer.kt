package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

@Config
class TwoDeadWheelLocalizer(
    hardwareMap: HardwareMap, private val imu: IMU, private val inPerTick: Double
) : IncrementalLocalizer {
    val par: Encoder
    val perp: Encoder

    private var lastParPos: Int
    private var lastPerpPos: Int
    private var lastHeading: Rotation2d

    private var lastRawHeadingVel = 0.0
    private var headingVelOffset = 0.0

    init {
        par = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "rightFront")))
        perp = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "rightBack")))

        par.direction = DcMotorSimple.Direction.REVERSE
        perp.direction = DcMotorSimple.Direction.REVERSE

        lastParPos = par.getPositionAndVelocity().position
        lastPerpPos = perp.getPositionAndVelocity().position
        lastHeading = Rotation2d.exp(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS)
    }

    private fun getHeadingVelocity(): Double {
        val rawHeadingVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()
        if (abs(rawHeadingVel - lastRawHeadingVel) > PI) {
            headingVelOffset -= sign(rawHeadingVel) * 2 * PI
        }
        lastRawHeadingVel = rawHeadingVel
        return headingVelOffset + rawHeadingVel
    }

    override fun update(): Twist2dDual<Time> {
        val parPosVel = par.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()
        val heading: Rotation2d =
            Rotation2d.exp(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

        val parPosDelta = parPosVel.position - lastParPos
        val perpPosDelta = perpPosVel.position - lastPerpPos
        val headingDelta = heading.minus(lastHeading)

        val headingVel = getHeadingVelocity()

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

        return twist
    }

    companion object {
        class Params {
            var parYTicks =
                7915.166024327065 // y position of the parallel encoder (in tick units)
            var perpXTicks =
                7592.816187212765 // x position of the perpendicular encoder (in tick units)
        }

        @JvmStatic
        var PARAMS = Params()
    }
}