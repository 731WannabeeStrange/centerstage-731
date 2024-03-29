package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

@Config
@TeleOp(group = "test")
public class DriveTestOpMode extends LinearOpMode {
    public static double DRIVE_FACTOR = 2;

    private final Pose2d startPose = new Pose2d(0, 0, -Math.PI / 2);
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

        MecanumDrive driveSubsystem = new MecanumDrive(hardwareMap, startPose, telemetryHandler);
        GamepadEx gamepad = new GamepadEx(gamepad1);
        driveSubsystem.setDefaultCommand(new ManualDriveCommand(driveSubsystem,
                () -> gamepad.getLeftX() / DRIVE_FACTOR,
                () -> gamepad.getLeftY() / DRIVE_FACTOR,
                () -> gamepad.getRightX() / DRIVE_FACTOR,
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT),
                ManualDriveCommand.FieldOrientation.BLUE,
                telemetryHandler));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            eTime.reset();
            CommandScheduler.getInstance().run();
            double elapsed = eTime.time();

            telemetryHandler.addData("Loop time", String.format("%.1f ms", elapsed));
            telemetryHandler.update();
        }

        CommandScheduler.getInstance().reset();
    }
}