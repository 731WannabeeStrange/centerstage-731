package org.firstinspires.ftc.teamcode.utils.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
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

@Config
class ThreeDeadWheelLocalizer(hardwareMap: HardwareMap, private val inPerTick: Double) :
    IncrementalLocalizer {
    val par0: Encoder
    val par1: Encoder
    val perp: Encoder

    private var lastPar0Pos: Double
    private var lastPar1Pos: Double
    private var lastPerpPos: Double

    init {
        par0 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "rightFront")))
        par1 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "rightOdo")))
        perp = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "rightBack")))

        par0.direction = DcMotorSimple.Direction.REVERSE
        perp.direction = DcMotorSimple.Direction.REVERSE

        lastPar0Pos = par0.getPositionAndVelocity().position.toDouble()
        lastPar1Pos = par1.getPositionAndVelocity().position.toDouble()
        lastPerpPos = perp.getPositionAndVelocity().position.toDouble()

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS)
    }

    override fun update(): Twist2dDual<Time> {
        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()

        val par0PosDelta = (par0PosVel.position - lastPar0Pos).toInt()
        val par1PosDelta = (par1PosVel.position - lastPar1Pos).toInt()
        val perpPosDelta = (perpPosVel.position - lastPerpPos).toInt()

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

        lastPar0Pos = par0PosVel.position.toDouble()
        lastPar1Pos = par1PosVel.position.toDouble()
        lastPerpPos = perpPosVel.position.toDouble()

        return twist
    }

    companion object {
        class Params {
            var par0YTicks =
                8144.3190903693585 // y position of the first parallel encoder (in tick units)
            var par1YTicks =
                -7352.5865316061445 // y position of the second parallel encoder (in tick units)
            var perpXTicks =
                7576.15139617889 // x position of the perpendicular encoder (in tick units)
        }

        @JvmStatic
        var PARAMS = Params()
    }
}