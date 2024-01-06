package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class ManualDriveCommand extends CommandBase {

    public static double P = 0.03;
    private final MecanumDrive drive;
    private final DoubleSupplier strafe, forward, rotation;
    private final BooleanSupplier autoTurn0, autoTurn90, autoTurn180, autoTurn270;
    private final TelemetryHandler telemetryHandler;
    private TurnState turnState = TurnState.DRIVER;
    private double desiredAngle = 0;

    public ManualDriveCommand(MecanumDrive drive, DoubleSupplier strafe, DoubleSupplier forward,
                              DoubleSupplier rotation, BooleanSupplier autoTurn0,
                              BooleanSupplier autoTurn90, BooleanSupplier autoTurn180,
                              BooleanSupplier autoTurn270, TelemetryHandler telemetryHandler) {
        this.drive = drive;
        this.strafe = strafe;
        this.forward = forward;
        this.rotation = rotation;
        this.autoTurn0 = autoTurn0;
        this.autoTurn90 = autoTurn90;
        this.autoTurn180 = autoTurn180;
        this.autoTurn270 = autoTurn270;
        this.telemetryHandler = telemetryHandler;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = drive.getPose().heading;
        Vector2d input = currentHeading.times(
                new Vector2d(-forward.getAsDouble(), -strafe.getAsDouble())
        );

        PoseVelocity2d powers;
        switch (turnState) {
            case DRIVER:
                powers = new PoseVelocity2d(input, rotation.getAsDouble());
                if (autoTurn0.getAsBoolean()) {
                    desiredAngle = 0;
                    turnState = TurnState.AUTO;
                } else if (autoTurn180.getAsBoolean()) {
                    desiredAngle = 180;
                    turnState = TurnState.AUTO;
                } else if (autoTurn90.getAsBoolean()) {
                    desiredAngle = -90;
                    turnState = TurnState.AUTO;
                } else if (autoTurn270.getAsBoolean()) {
                    desiredAngle = 90;
                    turnState = TurnState.AUTO;
                }
                break;
            case AUTO:
                if (autoTurn0.getAsBoolean()) {
                    desiredAngle = 0;
                } else if (autoTurn180.getAsBoolean()) {
                    desiredAngle = 180;
                } else if (autoTurn90.getAsBoolean()) {
                    desiredAngle = -90;
                } else if (autoTurn270.getAsBoolean()) {
                    desiredAngle = 90;
                }
                double error = desiredAngle - Math.toDegrees(currentHeading.toDouble());
                if (error > 180) {
                    error -= 360;
                } else if (error < -180) {
                    error += 360;
                }
                telemetryHandler.addData("desired angle", desiredAngle);
                telemetryHandler.addData("turn error", error);
                powers = new PoseVelocity2d(input, P * -error);
                if (rotation.getAsDouble() != 0) {
                    turnState = TurnState.DRIVER;
                }
                break;
            default:
                throw new RuntimeException("weird turn state");
        }

        drive.setDrivePowers(powers);
    }

    private enum TurnState {
        DRIVER,
        AUTO
    }
}
