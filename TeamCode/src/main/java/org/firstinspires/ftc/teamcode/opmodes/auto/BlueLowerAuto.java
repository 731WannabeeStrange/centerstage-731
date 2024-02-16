/*
package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsGroundCommand;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.Selected;
import org.firstinspires.ftc.teamcode.utils.TeamPropProcessor;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.LowerTeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(group = "comp", preselectTeleOp = "BlueTeleOp")
public class BlueLowerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-37.5, 63, -Math.PI / 2), telemetryHandler);
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);
        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap);

        Command rightCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(-40, 46), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-34, 36, 0), 0)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .setTangent(Math.PI / 2)
                //.splineToConstantHeading(new Vector2d(-42, 48), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-36, 60), Math.PI / 2)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(42, 60), 0)
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(42, 32), 0)
                .stopAndAdd(new ScorePixelsCommand(2000, scoringMech))
                .setTangent(Math.PI)
                .afterDisp(3, new ResetElevatorCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(38, 50), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(48, 60, -Math.PI / 2), 0)
                .build();

        Command middleCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(-40, 46), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-36, 36, Math.PI / 2), 0)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(-40, 42), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-42, 48, 0), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-36, 60), Math.PI / 2)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(42, 60), 0)
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(42, 38.5), 0)
                .stopAndAdd(new ScorePixelsCommand(2000, scoringMech))
                .setTangent(Math.PI)
                .afterDisp(3, new ResetElevatorCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(38, 50), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(48, 60, -Math.PI / 2), 0)
                .build();

        Command leftCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(-40, 46), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-34, 36, Math.PI), 0)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .setTangent(Math.PI)
                .splineToSplineHeading(new Pose2d(-42, 48, 0), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-36, 60), Math.PI / 2)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(42, 60), 0)
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(42, 42), 0)
                .stopAndAdd(new ScorePixelsCommand(2000, scoringMech))
                .setTangent(Math.PI)
                .afterDisp(3, new ResetElevatorCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(38, 50), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(48, 60, -Math.PI / 2), 0)
                .build();

        TeamPropProcessor teamPropProcessor = new LowerTeamPropProcessor();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(1280, 720))
                .build();

        Selected selected = Selected.NONE;
        while (opModeInInit()) {
            selected = teamPropProcessor.getSelection();
            telemetryHandler.addData("Selection", selected);
            telemetryHandler.addData("Camera state", visionPortal.getCameraState());
            telemetryHandler.update();
        }

        visionPortal.close();
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
            PoseStorage.currentPose = drive.getPose();
            telemetryHandler.update();
        }

        CommandScheduler.getInstance().reset();
    }
}

 */