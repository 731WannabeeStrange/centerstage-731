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

    public static double P = 0.02;
    public static double SLOW_MODE_FACTOR = 4;
    private final MecanumDrive drive;
    private final DoubleSupplier strafe, forward, rotation;
    private final BooleanSupplier slowMode, autoTurnUp, autoTurnLeft, autoTurnDown, autoTurnRight;
    private final TelemetryHandler telemetryHandler;
    private TurnState turnState = TurnState.DRIVER;
    private double desiredAngle = 0;

    public enum FieldOrientation {
        BLUE,
        RED
    }

    private final FieldOrientation fieldOrientation;

    public ManualDriveCommand(MecanumDrive drive, DoubleSupplier strafe, DoubleSupplier forward,
                              DoubleSupplier rotation, BooleanSupplier slowMode,
                              BooleanSupplier autoTurnUp, BooleanSupplier autoTurnLeft,
                              BooleanSupplier autoTurnDown, BooleanSupplier autoTurnRight,
                              FieldOrientation fieldOrientation, TelemetryHandler telemetryHandler) {
        this.drive = drive;
        this.strafe = strafe;
        this.forward = forward;
        this.rotation = rotation;
        this.slowMode = slowMode;
        this.autoTurnUp = autoTurnUp;
        this.autoTurnLeft = autoTurnLeft;
        this.autoTurnDown = autoTurnDown;
        this.autoTurnRight = autoTurnRight;
        this.fieldOrientation = fieldOrientation;
        this.telemetryHandler = telemetryHandler;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = drive.getPose().heading;
        switch (fieldOrientation) {
            case BLUE:
                currentHeading = currentHeading.plus(Math.PI / 2);
                break;
            case RED:
                currentHeading = currentHeading.plus(-Math.PI / 2);
                break;
        }
        Vector2d input = currentHeading.inverse().times(
                new Vector2d(forward.getAsDouble(), -strafe.getAsDouble())
        );

        if (autoTurnUp.getAsBoolean()) {
            desiredAngle = 0;
            turnState = TurnState.AUTO;
        } else if (autoTurnDown.getAsBoolean()) {
            desiredAngle = 180;
            turnState = TurnState.AUTO;
        } else if (autoTurnLeft.getAsBoolean()) {
            desiredAngle = 90;
            turnState = TurnState.AUTO;
        } else if (autoTurnRight.getAsBoolean()) {
            desiredAngle = 270;
            turnState = TurnState.AUTO;
        }
        PoseVelocity2d powers;
        switch (turnState) {
            case DRIVER:
                powers = new PoseVelocity2d(input, -rotation.getAsDouble());
                break;
            case AUTO:
                double error = desiredAngle - Math.toDegrees(currentHeading.toDouble());
                if (error > 180) {
                    error -= 360;
                } else if (error < -180) {
                    error += 360;
                }
                telemetryHandler.addData("desired angle", desiredAngle);
                telemetryHandler.addData("turn error", error);
                powers = new PoseVelocity2d(input, P * error);
                if (rotation.getAsDouble() != 0) {
                    turnState = TurnState.DRIVER;
                }
                break;
            default:
                throw new RuntimeException("weird turn state");
        }

        if (slowMode.getAsBoolean()) {
            powers = new PoseVelocity2d(powers.linearVel.div(SLOW_MODE_FACTOR), powers.angVel / SLOW_MODE_FACTOR);
        }

        drive.setDrivePowers(powers);
    }

    private enum TurnState {
        DRIVER,
        AUTO
    }
}
