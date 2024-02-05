/*
package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.FlushIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePixelsCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Autonomous(group = "test")
@Config
public class AutoIntakeTestOpMode extends LinearOpMode {
    public static double DISTANCE = 10;

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), telemetryHandler);
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);

        waitForStart();

        scheduler.schedule(
                drive.pathCommandBuilder(drive.pose)
                        .afterTime(0, new SequentialCommandGroup(
                                new IntakePixelsCommand(scoringMech),
                                new FlushIntakeCommand(scoringMech)
                        ))
                        .lineToX(-DISTANCE)
                        .build()
        );

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            telemetryHandler.update();
        }

        scheduler.reset();
    }
}


 */