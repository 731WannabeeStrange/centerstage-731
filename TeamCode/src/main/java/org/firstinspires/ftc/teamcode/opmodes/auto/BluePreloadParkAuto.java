/*
package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsGroundCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.vision.Selected;
import org.firstinspires.ftc.teamcode.utils.vision.UpperTeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(group = "comp", preselectTeleOp = "BlueTeleOp")
public class BluePreloadParkAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.5, 63, -Math.PI / 2), telemetryHandler);
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);

        Command rightCommand = drive.commandBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(15, 48, Math.toRadians(225)), -Math.PI / 2)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .splineToLinearHeading(new Pose2d(36, 28, 0), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new ScorePixelsCommand(2800, scoringMech),
                        new ResetElevatorCommand(scoringMech)
                ))
                .splineToLinearHeading(new Pose2d(52, 12, -Math.PI/2), 0)
                .build();

        Command middleCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(19, 44), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(23, 24, 0), -Math.PI / 2)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(43, 37.5), 0)
                .stopAndAdd(new ScorePixelsCommand(1600, scoringMech))
                .afterDisp(3, new ResetElevatorCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(38, 24), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(48, 15, -Math.PI/2), -Math.PI / 2)
                .build();

        Command leftCommand = drive.commandBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(13, 54, Math.toRadians(300)), -Math.PI / 2)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .splineToLinearHeading(new Pose2d(36, 42, 0), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new ScorePixelsCommand(2800, scoringMech),
                        new ResetElevatorCommand(scoringMech)
                ))
                .splineToLinearHeading(new Pose2d(52, 12, -Math.PI/2), 0)
                .build();

        UpperTeamPropProcessor teamPropProcessor = new UpperTeamPropProcessor();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(1280, 720))
                .build();

        Selected selected = Selected.NONE;
        while (opModeInInit()) {
            selected = teamPropProcessor.getSelection();
            telemetryHandler.addData("Selection", selected);
            telemetryHandler.update();
        }

        visionPortal.stopStreaming();
        switch (selected) {
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

 */