package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@TeleOp
public class CommandTeleOpTestOpMode extends LinearOpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private DriveSubsystem driveSubsystem;
    private GamepadEx gamepad;

    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();

    private final Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(hardwareMap, startPose);
        scheduler.registerSubsystem(driveSubsystem);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        driveSubsystem.setDefaultCommand(new ManualDriveCommand(driveSubsystem, gamepad::getLeftX, gamepad::getLeftY, gamepad::getRightX));

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            telemetryHandler.sendCurrentPacket();
        }

        scheduler.reset();
        telemetryHandler.reset();
    }
}
