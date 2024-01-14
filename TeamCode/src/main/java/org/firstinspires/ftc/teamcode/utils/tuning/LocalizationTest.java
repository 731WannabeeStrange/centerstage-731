package org.firstinspires.ftc.teamcode.utils.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), telemetryHandler);
        GamepadEx gamepad = new GamepadEx(gamepad1);
        drive.setDefaultCommand(new ManualDriveCommand(drive, gamepad::getLeftX,
                gamepad::getLeftY, gamepad::getRightX,
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
                ManualDriveCommand.FieldOrientation.BLUE,
                telemetryHandler));

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            eTime.reset();
            CommandScheduler.getInstance().run();
            double elapsed = eTime.time();

            Pose2d currentPose = drive.getPose();
            telemetryHandler.addData("x", currentPose.position.x);
            telemetryHandler.addData("y", currentPose.position.y);
            telemetryHandler.addData("heading", currentPose.heading);
            telemetryHandler.addData("Loop time", String.format("%.1f ms", elapsed));
            telemetryHandler.update();
        }

        CommandScheduler.getInstance().reset();
    }
}
