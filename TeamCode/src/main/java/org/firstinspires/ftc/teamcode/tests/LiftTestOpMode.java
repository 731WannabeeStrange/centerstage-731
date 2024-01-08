package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@TeleOp(group = "test")
public class LiftTestOpMode extends LinearOpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        Lift liftSubsystem = new Lift(hardwareMap, telemetryHandler);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        // fix with default commands
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenActive(new InstantCommand(() -> liftSubsystem.setLiftState(Lift.LiftState.OUTTAKE), liftSubsystem))
                .whenInactive(new InstantCommand(() -> liftSubsystem.setLiftState(Lift.LiftState.INTAKE), liftSubsystem));
        //liftSubsystem.setDefaultCommand(new PerpetualCommand(new InstantCommand(() -> liftSubsystem.setLiftState(Lift.LiftState.INTAKE), liftSubsystem)));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            scheduler.run();
            telemetryHandler.update();
        }

        scheduler.reset();
    }
}
