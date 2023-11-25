package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectoriesCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.BuilderUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

@Autonomous(group = "test")
public class CommandAutoTestOpMode extends LinearOpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private DriveSubsystem driveSubsystem;

    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();

    private final Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(hardwareMap, startPose);
        scheduler.registerSubsystem(driveSubsystem);

        List<Trajectory> trajectories = BuilderUtils.getTrajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toDegrees(90)), 0)
                .build();
        TimeTurn turn1 = BuilderUtils.turnBuilder(Math.toRadians(90), BuilderUtils.getEndPose(trajectories));
        TimeTurn turn2 = BuilderUtils.turnToBuilder(Math.toRadians(0), BuilderUtils.getEndPose(turn1));

        waitForStart();

        scheduler.schedule(
                new FollowTrajectoriesCommand(driveSubsystem, trajectories),
                new FollowTurnCommand(driveSubsystem, turn1),
                new WaitCommand(3000),
                new FollowTurnCommand(driveSubsystem, turn2)
        );

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            telemetryHandler.sendCurrentPacket();
        }

        scheduler.reset();
        telemetryHandler.reset();
    }
}
