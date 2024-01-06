package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Autonomous(group = "test")
public class CommandAutoTestOpMode extends LinearOpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
    private final Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive driveSubsystem = new MecanumDrive(hardwareMap, startPose, telemetryHandler);

        waitForStart();

        scheduler.schedule(
                driveSubsystem.commandBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), 0)
                        .turn(Math.toDegrees(90))
                        .waitSeconds(3)
                        .turnTo(Math.toRadians(0))
                        .build()
        );

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            telemetryHandler.update();
        }

        scheduler.reset();
    }
}
