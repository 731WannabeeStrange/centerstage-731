package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
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

@Config
class ThreeDeadWheelLocalizer(
    hardwareMap: HardwareMap,
    private val inPerTick: Double,
    override var poseEstimate: Pose2d
) : Localizer {
    @JvmField
    val par0: Encoder
    @JvmField
    val par1: Encoder
    @JvmField
    val perp: Encoder
    private var lastPar0Pos: Int
    private var lastPar1Pos: Int
    private var lastPerpPos: Int
    override var poseVelocity = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        private set

    init {
        par0 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "leftOuttake")))
        par1 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "rightOdo")))
        perp = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "frMotor")))
        lastPar0Pos = par0.getPositionAndVelocity().position
        lastPar1Pos = par1.getPositionAndVelocity().position
        lastPerpPos = perp.getPositionAndVelocity().position
        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS)
    }

    override fun update() {
        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()
        val par0PosDelta = par0PosVel.position - lastPar0Pos
        val par1PosDelta = par1PosVel.position - lastPar1Pos
        val perpPosDelta = perpPosVel.position - lastPerpPos
        val twist = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    listOf(
                        (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks)
                    )
                ).times(inPerTick),
                DualNum<Time>(
                    listOf(
                        PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta,
                        PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity
                    )
                ).times(inPerTick)
            ),
            DualNum(
                listOf(
                    (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks)
                )
            )
        )
        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
        lastPerpPos = perpPosVel.position
        poseEstimate = poseEstimate.plus(twist.value())
        poseVelocity = twist.velocity().value()
    }

    class Params {
        @JvmField
        var par0YTicks = 0.0 // y position of the first parallel encoder (in tick units)
        @JvmField
        var par1YTicks = 1.0 // y position of the second parallel encoder (in tick units)
        @JvmField
        var perpXTicks = 0.0 // x position of the perpendicular encoder (in tick units)
    }

    companion object {
        @JvmField
        var PARAMS = Params()
    }
}
