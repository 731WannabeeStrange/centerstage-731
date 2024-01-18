package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.commands.old.ManualScoringCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.old.Elevator;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

@TeleOp(group = "comp")
public class RedTeleOp extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(0, 0, Math.PI / 2);
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

        MecanumDrive driveSubsystem = new MecanumDrive(hardwareMap, startPose, telemetryHandler);
        Intake intakeSubsystem = new Intake(hardwareMap, telemetryHandler);
        Elevator elevatorSubsystem = new Elevator(hardwareMap, telemetryHandler);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        CommandScheduler.getInstance().schedule(new ManualScoringCommand(intakeSubsystem, elevatorSubsystem,
                () -> gamepad.getButton(GamepadKeys.Button.B),
                () -> gamepad.getButton(GamepadKeys.Button.X),
                () -> gamepad.getButton(GamepadKeys.Button.Y),
                () -> gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER),
                () -> gamepad1.rumble(500)));
        driveSubsystem.setDefaultCommand(new ManualDriveCommand(driveSubsystem, gamepad::getLeftX,
                gamepad::getLeftY, gamepad::getRightX,
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN),
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT),
                ManualDriveCommand.FieldOrientation.RED,
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