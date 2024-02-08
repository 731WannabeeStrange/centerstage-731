package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.FlushIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeFirstCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSecondCommand;
import org.firstinspires.ftc.teamcode.commands.KnockDownStackCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Autonomous(group = "test")
@Config
public class KnockDownStackTestOpMode extends LinearOpMode {
    public static double FIRST_HEIGHT = 0.62;
    public static double SECOND_HEIGHT = 0.64;
    public static double BACKWARD_DISTANCE = 0;
    public static double FORWARD_DISTANCE = 3;

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), telemetryHandler);
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);

        waitForStart();

        scheduler.schedule(
                drive.pathCommandBuilder(drive.pose)
                        .lineToX(-BACKWARD_DISTANCE)
                        .stopAndAdd(new KnockDownStackCommand(FIRST_HEIGHT, scoringMech))
                        .lineToX(FORWARD_DISTANCE)
                        .stopAndAdd(new IntakeFirstCommand(scoringMech))
                        .lineToX(-FORWARD_DISTANCE)
                        .stopAndAdd(new KnockDownStackCommand(SECOND_HEIGHT, scoringMech))
                        .lineToX(FORWARD_DISTANCE)
                        .stopAndAdd(new SequentialCommandGroup(
                                new IntakeSecondCommand(scoringMech),
                                new FlushIntakeCommand(scoringMech)
                        ))
                        .build()
        );

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            telemetryHandler.update();
        }

        scheduler.reset();
    }
}
