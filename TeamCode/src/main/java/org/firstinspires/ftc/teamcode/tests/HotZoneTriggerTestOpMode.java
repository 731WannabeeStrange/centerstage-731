package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.HotZoneTrigger;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

@TeleOp(group = "test")
public class HotZoneTriggerTestOpMode extends LinearOpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
    private final Pose2d startPose = new Pose2d(0, 0, 0);
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        MecanumDrive driveSubsystem = new MecanumDrive(hardwareMap, startPose, telemetryHandler);
        Intake intakeSubsystem = new Intake(hardwareMap, telemetryHandler);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        new HotZoneTrigger(driveSubsystem::getPose, new Vector2d(0, 72), new Vector2d(-72, -72))
                .whenActive(new InstantCommand(intakeSubsystem::start, intakeSubsystem))
                .whenInactive(new InstantCommand(intakeSubsystem::stop, intakeSubsystem));
        driveSubsystem.setDefaultCommand(new ManualDriveCommand(driveSubsystem, gamepad::getLeftX,
                gamepad::getLeftY, gamepad::getRightX,
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
                telemetryHandler));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            eTime.reset();
            scheduler.run();
            double elapsed = eTime.time();

            telemetryHandler.addData("Loop time", String.format("%.1f ms", elapsed));
            telemetryHandler.update();
        }

        scheduler.reset();
    }
}
