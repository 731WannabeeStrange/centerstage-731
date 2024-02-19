package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.IdentityPoseMap
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Rotation2dDual
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.map
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.ParallelRaceGroup
import com.arcrobotics.ftclib.command.ScheduleCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand

enum class ParallelType {
    NORMAL, INTERRUPT, FORK
}

private sealed class MarkerFactory(val segmentIndex: Int, val parallelType: ParallelType) {
    abstract fun make(t: TimeTrajectory, segmentDisp: Double): Command
}

private class TimeMarkerFactory(
    segmentIndex: Int,
    parallelType: ParallelType,
    val dt: Double,
    val c: Command
) :
    MarkerFactory(segmentIndex, parallelType) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        SequentialCommandGroup(
            WaitCommand(((t.profile.inverse(segmentDisp) + dt) * 1000).toLong()),
            c
        )
}

private class DispMarkerFactory(
    segmentIndex: Int,
    parallelType: ParallelType,
    val ds: Double,
    val c: Command
) :
    MarkerFactory(segmentIndex, parallelType) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        SequentialCommandGroup(
            WaitCommand(((t.profile.inverse(segmentDisp + ds)) * 1000).toLong()),
            c
        )
}

private class CommandList(
    val commands: List<Command> = listOf<Command>(),
    val parallels: List<Command> = listOf<Command>(),
    val interrupts: List<Command> = listOf<Command>(),
) {
    fun with(c: Command, parallelType: ParallelType): CommandList {
        return when (parallelType) {
            ParallelType.NORMAL -> CommandList(
                commands,
                parallels + c,
                interrupts
            )
            ParallelType.INTERRUPT -> CommandList(
                commands,
                parallels,
                interrupts + c
            )
            ParallelType.FORK -> CommandList(
                commands,
                parallels + ScheduleCommand(c),
                interrupts
            )
        }
    }

    operator fun plus(c: Command) = CommandList(
        commands + if (interrupts.isEmpty()) {
            if (parallels.isEmpty()) c else ParallelCommandGroup(*parallels.toTypedArray(), c)
        } else {
            ParallelDeadlineGroup(
                if (interrupts.size == 1) interrupts.first() else ParallelRaceGroup(*interrupts.toTypedArray()),
                *parallels.toTypedArray(),
                c
            )
        },
    )

    operator fun plus(cs: List<Command>) = CommandList(
        commands + if (interrupts.isEmpty()) {
            if (parallels.isEmpty()) cs.first() else ParallelCommandGroup(*parallels.toTypedArray(), cs.first())
        } else {
            ParallelDeadlineGroup(
                if (interrupts.size == 1) interrupts.first() else ParallelRaceGroup(*interrupts.toTypedArray()),
                *parallels.toTypedArray(),
                cs.first()
            )
        } + cs.drop(1)
    )
}

fun interface TurnCommandFactory {
    fun make(t: TimeTurn): Command
}

fun interface TrajectoryCommandFactory {
    fun make(t: DisplacementTrajectory): Command
}

class TrajectoryCommand(
    commandList: List<Command>,
    val endPose: Pose2d
) : SequentialCommandGroup(*commandList.toTypedArray())

class TrajectoryCommandBuilder private constructor(
    val turnCommandFactory: TurnCommandFactory,
    val trajectoryCommandFactory: TrajectoryCommandFactory,
    val eps: Double,
    val beginEndVel: Double,
    val baseTurnConstraints: TurnConstraints,
    val baseVelConstraint: VelConstraint,
    val baseAccelConstraint: AccelConstraint,
    val dispResolution: Double,
    val angResolution: Double,
    val poseMap: PoseMap,
    // vary throughout
    private val tb: TrajectoryBuilder,
    private val n: Int,
    // lastPose, lastTangent are post-mapped
    private val lastPoseUnmapped: Pose2d,
    private val lastPose: Pose2d,
    private val lastTangent: Rotation2d,
    private val ms: List<MarkerFactory>,
    private val commandList: CommandList
) {
    @JvmOverloads
    constructor(
        turnCommandFactory: TurnCommandFactory,
        trajectoryCommandFactory: TrajectoryCommandFactory,
        beginPose: Pose2d,
        eps: Double,
        beginEndVel: Double,
        baseTurnConstraints: TurnConstraints,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        dispResolution: Double,
        angResolution: Double,
        poseMap: PoseMap = IdentityPoseMap(),
    ) :
            this(
                turnCommandFactory,
                trajectoryCommandFactory,
                eps,
                beginEndVel,
                baseTurnConstraints,
                baseVelConstraint,
                baseAccelConstraint,
                dispResolution,
                angResolution,
                poseMap,
                TrajectoryBuilder(
                    beginPose, eps, beginEndVel,
                    baseVelConstraint, baseAccelConstraint, dispResolution, angResolution,
                    poseMap,
                ),
                0,
                beginPose,
                poseMap.map(beginPose),
                poseMap.map(beginPose).heading,
                emptyList(),
                CommandList()
            )

    private constructor(
        ab: TrajectoryCommandBuilder,
        tb: TrajectoryBuilder,
        n: Int,
        lastPoseUnmapped: Pose2d,
        lastPose: Pose2d,
        lastTangent: Rotation2d,
        ms: List<MarkerFactory>,
        commandList: CommandList
    ) :
            this(
                ab.turnCommandFactory,
                ab.trajectoryCommandFactory,
                ab.eps,
                ab.beginEndVel,
                ab.baseTurnConstraints,
                ab.baseVelConstraint,
                ab.baseAccelConstraint,
                ab.dispResolution,
                ab.angResolution,
                ab.poseMap,
                tb,
                n,
                lastPoseUnmapped,
                lastPose,
                lastTangent,
                ms,
                commandList
            )

    /**
     * Ends the current trajectory in progress. No-op if no trajectory segments are pending.
     */
    fun endTrajectory() =
        if (n == 0) {
            require(ms.isEmpty())

            this
        } else {
            val ts = tb.build()
            val endPoseUnmapped = ts.last().path.basePath.end(1).value()
            val end = ts.last().path.end(2)
            val endPose = end.value()
            val endTangent = end.velocity().value().linearVel.angleCast()
            val (newCommands, msRem) = ts.zip(ts.scan(0) { acc, t -> acc + t.offsets.size })
                .foldRightIndexed(
                    Pair(emptyList<Command>(), ms)
                ) { index, (traj, offset), (acc, ms) ->
                    val timeTraj = TimeTrajectory(traj)
                    val dispTraj = DisplacementTrajectory(traj)
                    val commands: MutableList<Command> = mutableListOf(
                        trajectoryCommandFactory.make(dispTraj)
                    )

                    val interruptors = mutableListOf<Command>()
                    val msRem = mutableListOf<MarkerFactory>()
                    for (m in ms) {
                        val i = m.segmentIndex - offset + index
                        if (i >= 0) {
                            when (m.parallelType) {
                                ParallelType.NORMAL -> commands.add(m.make(timeTraj, traj.offsets[i]))
                                ParallelType.INTERRUPT -> interruptors.add(m.make(timeTraj, traj.offsets[i]))
                                ParallelType.FORK -> commands.add(ScheduleCommand(m.make(timeTraj, traj.offsets[i])))
                            }
                        } else {
                            msRem.add(m)
                        }
                    }

                    if (interruptors.size > 0) {
                        Pair(
                            listOf(
                                ParallelDeadlineGroup(
                                    if (interruptors.size > 1) ParallelRaceGroup(*interruptors.toTypedArray()) else interruptors.first(),
                                    *commands.toTypedArray()
                                )
                            ) + acc, msRem
                        )
                    } else {
                        Pair(
                            listOf(
                                if (commands.size == 1) commands.first() else ParallelCommandGroup(*commands.toTypedArray())
                            ) + acc, msRem
                        )
                    }
                }

            require(msRem.isEmpty())

            TrajectoryCommandBuilder(
                this,
                TrajectoryBuilder(
                    endPoseUnmapped,
                    eps,
                    beginEndVel,
                    baseVelConstraint,
                    baseAccelConstraint,
                    dispResolution, angResolution,
                    poseMap,
                ),
                0,
                endPoseUnmapped,
                endPose,
                endTangent,
                emptyList(),
                commandList + newCommands
            )
        }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds command [c] next.
     */
    fun stopAndAdd(c: Command): TrajectoryCommandBuilder {
        val b = endTrajectory()
        return TrajectoryCommandBuilder(
                b,
                b.tb,
                b.n,
                b.lastPoseUnmapped,
                b.lastPose,
                b.lastTangent,
                b.ms,
                b.commandList + c
            )
    }

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double): TrajectoryCommandBuilder {
        require(t >= 0.0)

        return stopAndAdd(WaitCommand((t * 1000).toLong()))
    }

    /**
     * Schedules command [c] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    @JvmOverloads
    fun afterDisp(ds: Double, c: Command, parallelType: ParallelType = ParallelType.NORMAL): TrajectoryCommandBuilder {
        require(ds >= 0.0)

        return TrajectoryCommandBuilder(
            this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
            ms + listOf(DispMarkerFactory(n, parallelType, ds, c)), commandList
        )
    }

    /**
     * Schedules command [c] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other command.
     */
    @JvmOverloads
    fun afterTime(dt: Double, c: Command, parallelType: ParallelType = ParallelType.NORMAL): TrajectoryCommandBuilder {
        require(dt >= 0.0)

        return if (n == 0) {
            TrajectoryCommandBuilder(
                this, tb, 0, lastPoseUnmapped, lastPose, lastTangent, ms,
                commandList.with(SequentialCommandGroup(WaitCommand((dt*1000).toLong()), c), parallelType)
            )
        } else {
            TrajectoryCommandBuilder(
                this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
                ms + listOf(TimeMarkerFactory(n, parallelType, dt, c)), commandList
            )
        }
    }

    fun setTangent(r: Rotation2d) =
        TrajectoryCommandBuilder(
            this,
            tb.setTangent(r),
            n,
            lastPoseUnmapped,
            lastPose,
            lastTangent,
            ms,
            commandList
        )

    fun setTangent(r: Double) = setTangent(Rotation2d.exp(r))

    fun setReversed(reversed: Boolean) =
        TrajectoryCommandBuilder(
            this,
            tb.setReversed(reversed),
            n,
            lastPoseUnmapped,
            lastPose,
            lastTangent,
            ms,
            commandList
        )

    @JvmOverloads
    fun turn(
        angle: Double,
        turnConstraintsOverride: TurnConstraints? = null
    ): TrajectoryCommandBuilder {
        val b = endTrajectory()
        val mappedAngle =
            poseMap.map(
                Pose2dDual(
                    Vector2dDual.constant(b.lastPose.position, 2),
                    Rotation2dDual.constant<Arclength>(b.lastPose.heading, 2) + DualNum(
                        listOf(
                            0.0,
                            angle
                        )
                    )
                )
            ).heading.velocity().value()
        val b2 = b.stopAndAdd(
            turnCommandFactory.make(
                TimeTurn(b.lastPose, mappedAngle, turnConstraintsOverride ?: baseTurnConstraints)
            )
        )
        val lastPoseUnmapped =
            Pose2d(b2.lastPoseUnmapped.position, b2.lastPoseUnmapped.heading + angle)
        val lastPose = Pose2d(b2.lastPose.position, b2.lastPose.heading + mappedAngle)
        val lastTangent = b2.lastTangent + mappedAngle
        return TrajectoryCommandBuilder(
            b2,
            TrajectoryBuilder(
                lastPoseUnmapped,
                eps,
                beginEndVel,
                baseVelConstraint,
                baseAccelConstraint,
                dispResolution, angResolution,
                poseMap
            ),
            b2.n, lastPoseUnmapped, lastPose, lastTangent, b2.ms, b2.commandList
        )
    }

    @JvmOverloads
    fun turnTo(
        heading: Rotation2d,
        turnConstraintsOverride: TurnConstraints? = null
    ): TrajectoryCommandBuilder {
        val b = endTrajectory()
        return b.turn(heading - b.lastPose.heading, turnConstraintsOverride)
    }

    @JvmOverloads
    fun turnTo(heading: Double, turnConstraintsOverride: TurnConstraints? = null) =
        turnTo(Rotation2d.exp(heading), turnConstraintsOverride)

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToX(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToY(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeTo(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToConstantHeading(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, commandList
    )

    /**
     * Creates a new builder with the same settings at the current pose, tangent.
     */
    fun fresh() = TrajectoryCommandBuilder(
        turnCommandFactory,
        trajectoryCommandFactory,
        lastPoseUnmapped,
        eps, beginEndVel, baseTurnConstraints, baseVelConstraint, baseAccelConstraint,
        dispResolution, angResolution, poseMap
    ).setTangent(lastTangent)

    fun build(): TrajectoryCommand {
        val b = endTrajectory().stopAndAdd(SequentialCommandGroup())
        return TrajectoryCommand(b.commandList.commands, b.lastPose)
    }
}