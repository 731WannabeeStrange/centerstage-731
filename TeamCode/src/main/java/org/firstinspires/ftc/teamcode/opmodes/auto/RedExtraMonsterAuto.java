package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.LowerTeamPropProcessor;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.Selected;
import org.firstinspires.ftc.teamcode.utils.TeamPropProcessor;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(group = "comp", preselectTeleOp = "RedTeleOp")
public class RedExtraMonsterAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14.5, -63, Math.PI / 2), telemetryHandler);
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);
        DroneLauncher drone = new DroneLauncher(hardwareMap, telemetryHandler);

        TurnConstraints slowTurnConstraints = new TurnConstraints(Math.PI / 3, -Math.PI, Math.PI);
        Command rightCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(36, -42), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(33, -34, 0), Math.PI / 2)
                .afterTime(0.2, scoringMech.raiseElevator(1800))
                .stopAndAdd(scoringMech.scoreGround())
                .splineToConstantHeading(new Vector2d(43, -43.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(0.5)
                ))
                .afterDisp(0, scoringMech.resetElevator())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, -12.5), -Math.PI,
                        (pose2dDual, posePath, v) -> 35,
                        (pose2dDual, posePath, v) -> new MinMax(-20, 35))
                .lineToX(-42)
                .afterDisp(3, scoringMech.dropIntake())
                .splineToSplineHeading(new Pose2d(-56, -12.5, Math.toRadians(-22)), -Math.PI)
                .turn(Math.PI / 3, slowTurnConstraints)
                .stopAndAdd(scoringMech.spinIntake(1.5))
                .afterDisp(0, scoringMech.flushIntake())
                .splineToSplineHeading(new Pose2d(-42, -12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, -12.5), 0)
                .afterTime(0, scoringMech.raiseElevator(2600))
                .splineToConstantHeading(new Vector2d(39, -32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(1)
                ))
                .afterDisp(0, scoringMech.resetElevator())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, -12.5), -Math.PI,
                        (pose2dDual, posePath, v) -> 35,
                        (pose2dDual, posePath, v) -> new MinMax(-20, 35))
                .afterDisp(20, new SequentialCommandGroup(
                        scoringMech.dropIntake(),
                        scoringMech.spinIntake()
                ))
                .lineToX(-42)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-56.5, -17), -Math.PI)
                .waitSeconds(0.15)
                .afterDisp(0, scoringMech.flushIntake())
                .splineToSplineHeading(new Pose2d(-42, -12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, -12.5), 0)
                .afterTime(0.1, scoringMech.raiseElevator(2600))
                .splineToConstantHeading(new Vector2d(37, -32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(1),
                        scoringMech.resetElevator()
                ))
                .build();

        Command middleCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(24, -44), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(26, -26, 0), Math.PI / 2)
                .afterTime(0.2, scoringMech.raiseElevator(1800))
                .stopAndAdd(scoringMech.scoreGround())
                .splineToConstantHeading(new Vector2d(43, -37.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(0.5)
                ))
                .afterDisp(0, scoringMech.resetElevator())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, -12.5), -Math.PI,
                        (pose2dDual, posePath, v) -> 35,
                        (pose2dDual, posePath, v) -> new MinMax(-20, 35))
                .lineToX(-42)
                .afterDisp(3, scoringMech.dropIntake())
                .splineToSplineHeading(new Pose2d(-56, -12.5, Math.toRadians(-22)), -Math.PI)
                .turn(Math.PI / 3, slowTurnConstraints)
                .stopAndAdd(scoringMech.spinIntake(1.5))
                .afterDisp(0, scoringMech.flushIntake())
                .splineToSplineHeading(new Pose2d(-42, -12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, -12.5), 0)
                .afterTime(0, scoringMech.raiseElevator(2600))
                .splineToConstantHeading(new Vector2d(39, -32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(1)
                ))
                .afterDisp(0, scoringMech.resetElevator())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, -12.5), -Math.PI,
                        (pose2dDual, posePath, v) -> 35,
                        (pose2dDual, posePath, v) -> new MinMax(-20, 35))
                .afterDisp(20, new SequentialCommandGroup(
                        scoringMech.dropIntake(),
                        scoringMech.spinIntake()
                ))
                .lineToX(-42)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-56.5, -17), -Math.PI)
                .waitSeconds(0.15)
                .afterDisp(0, scoringMech.flushIntake())
                .splineToSplineHeading(new Pose2d(-42, -12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, -12.5), 0)
                .afterTime(0.1, scoringMech.raiseElevator(2600))
                .splineToConstantHeading(new Vector2d(37, -32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(1),
                        scoringMech.resetElevator()
                ))
                .build();

        Command leftCommand = drive.pathCommandBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(17, -44), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(10, -34, 0), Math.PI / 2)
                .afterTime(0.2, scoringMech.raiseElevator(1800))
                .stopAndAdd(scoringMech.scoreGround())
                .splineToConstantHeading(new Vector2d(43, -32), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(0.5)
                ))
                .afterDisp(0, scoringMech.resetElevator())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, -12.5), -Math.PI,
                        (pose2dDual, posePath, v) -> 35,
                        (pose2dDual, posePath, v) -> new MinMax(-20, 35))
                .lineToX(-42)
                .afterDisp(3, scoringMech.dropIntake())
                .splineToSplineHeading(new Pose2d(-56, -12.5, Math.toRadians(-22)), -Math.PI)
                .turn(Math.PI / 3, slowTurnConstraints)
                .stopAndAdd(scoringMech.spinIntake(1.5))
                .afterDisp(0, scoringMech.flushIntake())
                .splineToSplineHeading(new Pose2d(-42, -12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, -12.5), 0)
                .afterTime(0, scoringMech.raiseElevator(2600))
                .splineToConstantHeading(new Vector2d(39, -42.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(1)
                ))
                .afterDisp(0, scoringMech.resetElevator())
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, -12.5), -Math.PI,
                        (pose2dDual, posePath, v) -> 35,
                        (pose2dDual, posePath, v) -> new MinMax(-20, 35))
                .afterDisp(20, new SequentialCommandGroup(
                        scoringMech.dropIntake(),
                        scoringMech.spinIntake()
                ))
                .lineToX(-42)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-56.5, -17), -Math.PI)
                .waitSeconds(0.15)
                .afterDisp(0, scoringMech.flushIntake())
                .splineToSplineHeading(new Pose2d(-42, -12.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, -12.5), 0)
                .afterTime(0.1, scoringMech.raiseElevator(2600))
                .splineToConstantHeading(new Vector2d(37, -40.5), 0)
                .stopAndAdd(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !scoringMech.isElevatorBusy()),
                        scoringMech.releasePixels(1),
                        scoringMech.resetElevator()
                ))
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
