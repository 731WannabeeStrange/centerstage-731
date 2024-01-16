package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(group = "comp", preselectTeleOp = "RedTeleOp")
public class RedPreloadParkAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.5, -63, Math.PI / 2), telemetryHandler);
        Elevator elevator = new Elevator(hardwareMap, telemetryHandler);

        Command rightCommand = drive.commandBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(15, -48, -Math.toRadians(225)), Math.PI / 2)
                .stopAndAdd(new ScorePixelsCommand(elevator, Elevator.ElevatorState.MINIMUM))
                .splineToLinearHeading(new Pose2d(36, -28, 0), 0)
                .stopAndAdd(new ScorePixelsCommand(elevator, Elevator.ElevatorState.MAXIMUM))
                .splineToLinearHeading(new Pose2d(52, -12, Math.PI/2), 0)
                .afterTime(0, new ResetElevatorCommand(elevator))
                .build();

        Command middleCommand = drive.commandBuilder(drive.pose)
                .lineToYConstantHeading(-48)
                .stopAndAdd(new ScorePixelsCommand(elevator, Elevator.ElevatorState.MINIMUM))
                .splineToLinearHeading(new Pose2d(36, -34, 0), 0)
                .stopAndAdd(new ScorePixelsCommand(elevator, Elevator.ElevatorState.MAXIMUM))
                .splineToLinearHeading(new Pose2d(52, -12, Math.PI/2), 0)
                .afterTime(0, new ResetElevatorCommand(elevator))
                .build();

        Command leftCommand = drive.commandBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(13, -54, -Math.toRadians(300)), Math.PI / 2)
                .stopAndAdd(new ScorePixelsCommand(elevator, Elevator.ElevatorState.MINIMUM))
                .splineToLinearHeading(new Pose2d(36, -42, 0), 0)
                .stopAndAdd(new ScorePixelsCommand(elevator, Elevator.ElevatorState.MAXIMUM))
                .splineToLinearHeading(new Pose2d(52, -12, Math.PI/2), 0)
                .afterTime(0, new ResetElevatorCommand(elevator))
                .build();

        TeamPropProcessor teamPropProcessor = new TeamPropProcessor();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(1280, 720))
                .build();

        while (opModeInInit()) {
            telemetryHandler.addData("Selection", teamPropProcessor.getSelection());
            telemetryHandler.update();
        }

        switch (teamPropProcessor.getSelection()) {
            case NONE:
            case RIGHT:
                CommandScheduler.getInstance().schedule(rightCommand);
                break;
            case LEFT:
                CommandScheduler.getInstance().schedule(leftCommand);
                break;
            case MIDDLE:
                CommandScheduler.getInstance().schedule(middleCommand);
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            telemetryHandler.update();
        }

        CommandScheduler.getInstance().reset();
    }
}
