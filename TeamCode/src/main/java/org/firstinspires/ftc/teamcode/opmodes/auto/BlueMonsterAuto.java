package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.FlushIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeFirstCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePixelsCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSecondCommand;
import org.firstinspires.ftc.teamcode.commands.KnockDownStackCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.ReleasePixelsCommand;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsCommand;
import org.firstinspires.ftc.teamcode.commands.ScorePixelsGroundCommand;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.vision.Selected;
import org.firstinspires.ftc.teamcode.utils.vision.UpperTeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(group = "comp", preselectTeleOp = "BlueTeleOp")
public class BlueMonsterAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14.5, 63, -Math.PI / 2), telemetryHandler);
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);
        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap);

        TurnConstraints slowTurnConstraints = new TurnConstraints(Math.PI / 3, -Math.PI, Math.PI);
        Command rightCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(19, 44), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(10, 34, 0), -Math.PI)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .afterDisp(0, new RaiseElevatorCommand(1800, scoringMech))
                .splineToConstantHeading(new Vector2d(43, 32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(0.5, scoringMech)
                ))
                .afterDisp(0, new ResetElevatorCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-32, 12.5), -Math.PI)
                .splineToSplineHeading(new Pose2d(-56, 12.5, Math.toRadians(45)), -Math.PI)
                .afterTime(0, new IntakePixelsCommand(scoringMech))
                .turn(-Math.PI / 2, slowTurnConstraints)
                .afterDisp(0, new FlushIntakeCommand(scoringMech))
                .splineToSplineHeading(new Pose2d(-32, 12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .afterTime(0.1, new RaiseElevatorCommand(2600, scoringMech))
                .splineToConstantHeading(new Vector2d(38, 42.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(1, scoringMech)
                ))
                .afterDisp(0, new ResetElevatorCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-32, 12.5), -Math.PI)
                .afterDisp(0, new IntakePixelsCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-56.5, 15), -Math.PI)
                .waitSeconds(0.5)
                .afterDisp(0, new FlushIntakeCommand(scoringMech))
                .splineToSplineHeading(new Pose2d(-32, 12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .afterTime(0.1, new RaiseElevatorCommand(2600, scoringMech))
                .splineToConstantHeading(new Vector2d(38, 42.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(1, scoringMech),
                        new ResetElevatorCommand(scoringMech)
                ))
                .build();

        Command middleCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(19, 44), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(27, 26, 0), -Math.PI / 2)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .afterDisp(0, new RaiseElevatorCommand(1800, scoringMech))
                .splineToConstantHeading(new Vector2d(43, 38.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(0.5, scoringMech)
                ))
                .afterDisp(0, new ResetElevatorCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-32, 12.5), -Math.PI)
                .splineToSplineHeading(new Pose2d(-56, 12.5, Math.toRadians(45)), -Math.PI)
                .afterTime(0, new IntakePixelsCommand(scoringMech))
                .turn(-Math.PI / 2, slowTurnConstraints)
                .waitSeconds(0.5)
                .afterDisp(0, new FlushIntakeCommand(scoringMech))
                .splineToSplineHeading(new Pose2d(-32, 12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .afterTime(0.1, new RaiseElevatorCommand(2600, scoringMech))
                .splineToConstantHeading(new Vector2d(38, 32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(1, scoringMech)
                ))
                .afterDisp(0, new ResetElevatorCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-32, 12.5), -Math.PI)
                .afterDisp(0, new IntakePixelsCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-56.5, 15), -Math.PI)
                .waitSeconds(0.5)
                .afterDisp(0, new FlushIntakeCommand(scoringMech))
                .splineToSplineHeading(new Pose2d(-32, 12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .afterTime(0.1, new RaiseElevatorCommand(2600, scoringMech))
                .splineToConstantHeading(new Vector2d(38, 32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(1, scoringMech),
                        new ResetElevatorCommand(scoringMech)
                ))
                .build();

        Command leftCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(36, 42), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(33, 34, 0), -Math.PI / 2)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .afterDisp(0, new RaiseElevatorCommand(1800, scoringMech))
                .splineToConstantHeading(new Vector2d(43, 43.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(0.5, scoringMech)
                ))
                .afterDisp(0, new ResetElevatorCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-32, 12.5), -Math.PI)
                .splineToSplineHeading(new Pose2d(-56, 12.5, Math.toRadians(45)), -Math.PI)
                .afterTime(0, new IntakePixelsCommand(scoringMech))
                .turn(-Math.PI / 2, slowTurnConstraints)
                .waitSeconds(0.1)
                .afterDisp(0, new FlushIntakeCommand(scoringMech))
                .splineToSplineHeading(new Pose2d(-32, 12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .afterTime(0.1, new RaiseElevatorCommand(2600, scoringMech))
                .splineToConstantHeading(new Vector2d(38, 32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(1, scoringMech)
                ))
                .afterDisp(0, new ResetElevatorCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-32, 12.5), -Math.PI)
                .afterDisp(0, new IntakePixelsCommand(scoringMech))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-56.5, 15), -Math.PI)
                .waitSeconds(0.5)
                .afterDisp(0, new FlushIntakeCommand(scoringMech))
                .splineToSplineHeading(new Pose2d(-32, 12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .afterTime(0.1, new RaiseElevatorCommand(2600, scoringMech))
                .splineToConstantHeading(new Vector2d(38, 32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        new ReleasePixelsCommand(1, scoringMech),
                        new ResetElevatorCommand(scoringMech)
                ))
                .build();

        UpperTeamPropProcessor teamPropProcessor = new UpperTeamPropProcessor();
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
