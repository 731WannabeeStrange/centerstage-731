package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsGroundCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Autonomous(group = "test")
public class CommandAutoTestOpMode extends LinearOpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
    private final Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive driveSubsystem = new MecanumDrive(hardwareMap, startPose, telemetryHandler);
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);

        waitForStart();

        scheduler.schedule(
                driveSubsystem.commandBuilder(startPose)
                        .lineToXLinearHeading(20, Math.toRadians(90))
                        .waitSeconds(3)
                        .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                        .waitSeconds(1)
                        .stopAndAdd(new SequentialCommandGroup(
                                new ScorePixelsCommand(2800, scoringMech),
                                new ResetElevatorCommand(scoringMech)
                        ))
                        .turnTo(0)
                        .build()
        );

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            telemetryHandler.update();
        }

        scheduler.reset();
    }
}
