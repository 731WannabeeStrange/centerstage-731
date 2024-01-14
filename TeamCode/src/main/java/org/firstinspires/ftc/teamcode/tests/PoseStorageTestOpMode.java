package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

@Autonomous(group = "test", preselectTeleOp = "BlueTeleOp")
public class PoseStorageTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, -Math.PI / 2), telemetryHandler);

        Command followCommand = drive.commandBuilder(drive.pose)
                .lineToYLinearHeading(-20, 0)
                .build();
        CommandScheduler.getInstance().schedule(followCommand);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            PoseStorage.currentPose = drive.getPose();
        }

        PoseStorage.currentPose = drive.getPose();
        CommandScheduler.getInstance().reset();
    }
}
