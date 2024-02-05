package org.firstinspires.ftc.teamcode.utils.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), telemetryHandler);
        CommandScheduler.getInstance().schedule(drive.commandBuilder(drive.getPose())
                .splineTo(new Vector2d(15, 15), Math.PI / 2)
                .splineTo(new Vector2d(0, 30), Math.PI)
                .build());

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            telemetryHandler.update();
        }

        CommandScheduler.getInstance().reset();
    }
}
