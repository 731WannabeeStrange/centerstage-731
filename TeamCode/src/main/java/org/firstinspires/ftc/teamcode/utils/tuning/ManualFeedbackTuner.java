package org.firstinspires.ftc.teamcode.utils.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), telemetryHandler);

        Command followCommand = drive.commandBuilder(new Pose2d(0, 0, 0))
                .lineToX(DISTANCE)
                .lineToX(0)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            if (!CommandScheduler.getInstance().isScheduled(followCommand)) {
                followCommand = drive.commandBuilder(new Pose2d(0, 0, 0))
                        .lineToX(DISTANCE)
                        .lineToX(0)
                        .build();
                CommandScheduler.getInstance().schedule(followCommand);
            }

            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }
}
