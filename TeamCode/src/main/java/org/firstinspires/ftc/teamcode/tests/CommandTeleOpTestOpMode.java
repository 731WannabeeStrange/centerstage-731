package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@TeleOp(group = "test")
public class CommandTeleOpTestOpMode extends OpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();
    private final Pose2d startPose = new Pose2d(0, 0, 0);
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private GamepadEx gamepad;

    @Override
    public void init() {
        driveSubsystem = new DriveSubsystem(hardwareMap, startPose);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        new Trigger(new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)::isDown)
                .whenActive(new InstantCommand(intakeSubsystem::start, intakeSubsystem))
                .whenInactive(new InstantCommand(intakeSubsystem::stop, intakeSubsystem));
        driveSubsystem.setDefaultCommand(new ManualDriveCommand(driveSubsystem, gamepad::getLeftX,
                gamepad::getLeftY, gamepad::getRightX,
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)));
        intakeSubsystem.setDefaultCommand(new InstantCommand(intakeSubsystem::stop, intakeSubsystem));
    }

    @Override
    public void loop() {
        scheduler.run();
        telemetryHandler.sendCurrentPacket();
    }

    @Override
    public void stop() {
        scheduler.reset();
        telemetryHandler.reset();
    }
}
