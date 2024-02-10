package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.FlushIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeFirstCommand;
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
import org.firstinspires.ftc.teamcode.utils.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.utils.vision.Selected;
import org.firstinspires.ftc.teamcode.utils.vision.UpperTeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.HashMap;
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

        UpperTeamPropProcessor teamPropProcessor = new UpperTeamPropProcessor();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(1280, 720))
                .build();
        Selected selected = Selected.NONE;

        TrajectoryCommand leftPreloadCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(36, 42), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(33, 34, 0), -Math.PI / 2)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(43, 43.5), 0)
                .stopAndAdd(new ScorePixelsCommand(1800, scoringMech))
                .build();
        TrajectoryCommand middlePreloadCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(19, 44), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(27, 26, 0), -Math.PI / 2)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(43, 38.5), 0)
                .stopAndAdd(new ScorePixelsCommand(1800, scoringMech))
                .build();
        TrajectoryCommand rightPreloadCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(19, 44), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(10, 34, 0), -Math.PI)
                .stopAndAdd(new ScorePixelsGroundCommand(scoringMech))
                .splineToConstantHeading(new Vector2d(43, 32), 0)
                .stopAndAdd(new ScorePixelsCommand(1800, scoringMech))
                .build();

        TrajectoryCommand leftResetCommand = drive.pathCommandBuilder(leftPreloadCommand.getEndPose())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-56, 12.5), -Math.PI)
                .build();
        TrajectoryCommand middleResetCommand = drive.pathCommandBuilder(middlePreloadCommand.getEndPose())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-56, 12.5), -Math.PI)
                .build();
        TrajectoryCommand rightResetCommand = drive.pathCommandBuilder(rightPreloadCommand.getEndPose())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(30, 12.5), -Math.PI)
                .splineToConstantHeading(new Vector2d(-56, 12.5), -Math.PI)
                .build();

        TrajectoryCommand intakeFirstCommand = drive.pathCommandBuilder(leftResetCommand.getEndPose())
                .stopAndAdd(new KnockDownStackCommand(0.62, scoringMech))
                .lineToX(-54)
                .stopAndAdd(new IntakeFirstCommand(scoringMech))
                .lineToX(-57)
                .stopAndAdd(new KnockDownStackCommand(0.655, scoringMech))
                .lineToX(-54)
                .stopAndAdd(new IntakeSecondCommand(scoringMech))
                .build();

        TrajectoryCommand leftFirstCycleCommand = drive.pathCommandBuilder(intakeFirstCommand.getEndPose())
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .splineToConstantHeading(new Vector2d(38, 37), 0)
                .build();
        TrajectoryCommand middleFirstCycleCommand = drive.pathCommandBuilder(intakeFirstCommand.getEndPose())
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .splineToConstantHeading(new Vector2d(38, 42.5), 0)
                .build();
        TrajectoryCommand rightFirstCycleCommand = drive.pathCommandBuilder(intakeFirstCommand.getEndPose())
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .splineToConstantHeading(new Vector2d(38, 42.5), 0)
                .build();

        while (opModeInInit()) {
            selected = teamPropProcessor.getSelection();
            telemetryHandler.addData("Selection", selected);
            telemetryHandler.addData("Camera state", visionPortal.getCameraState());
            telemetryHandler.update();
        }

        visionPortal.close();
        if (selected == Selected.NONE) {
            selected = Selected.RIGHT;
        }
        Selected finalSelected = selected;

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(Selected.LEFT, leftPreloadCommand);
                                    put(Selected.MIDDLE, middlePreloadCommand);
                                    put(Selected.RIGHT, rightPreloadCommand);
                                }},
                                () -> finalSelected
                        ),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(Selected.LEFT, new ParallelCommandGroup(
                                            leftResetCommand,
                                            new SequentialCommandGroup(
                                                    new WaitCommand(500),
                                                    new ResetElevatorCommand(scoringMech)
                                            )
                                    ));
                                    put(Selected.MIDDLE, new ParallelCommandGroup(
                                            middleResetCommand,
                                            new SequentialCommandGroup(
                                                    new WaitCommand(500),
                                                    new ResetElevatorCommand(scoringMech)
                                            )
                                    ));
                                    put(Selected.RIGHT, new ParallelCommandGroup(
                                            rightResetCommand,
                                            new SequentialCommandGroup(
                                                    new WaitCommand(500),
                                                    new ResetElevatorCommand(scoringMech)
                                            )
                                    ));
                                }},
                                () -> finalSelected
                        ),
                        intakeFirstCommand,
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(Selected.LEFT, new ParallelCommandGroup(
                                            leftFirstCycleCommand,
                                            new SequentialCommandGroup(
                                                    new FlushIntakeCommand(scoringMech),
                                                    new WaitCommand(10000),
                                                    new RaiseElevatorCommand(2400, scoringMech)
                                            )
                                    ));
                                    put(Selected.MIDDLE, new ParallelCommandGroup(
                                            middleFirstCycleCommand,
                                            new SequentialCommandGroup(
                                                    new FlushIntakeCommand(scoringMech),
                                                    new WaitCommand(10000),
                                                    new RaiseElevatorCommand(2400, scoringMech)
                                            )
                                    ));
                                    put(Selected.RIGHT, new ParallelCommandGroup(
                                            rightFirstCycleCommand,
                                            new SequentialCommandGroup(
                                                    new FlushIntakeCommand(scoringMech),
                                                    new WaitCommand(10000),
                                                    new RaiseElevatorCommand(2400, scoringMech)
                                            )
                                    ));
                                }},
                                () -> finalSelected
                        ),
                        new ReleasePixelsCommand(2.0, scoringMech),
                        new ResetElevatorCommand(scoringMech)
                )
        );

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
