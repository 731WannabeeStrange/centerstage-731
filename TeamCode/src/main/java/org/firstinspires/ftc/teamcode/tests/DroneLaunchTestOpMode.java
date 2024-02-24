package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Config
@TeleOp
public class DroneLaunchTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);
        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap, telemetryHandler);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.BACK).and(gamepad.getGamepadButton(GamepadKeys.Button.START))
                .whenActive(droneLauncher.launchDrone());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetryHandler.update();
        }

        CommandScheduler.getInstance().reset();
    }
}
