package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
public class ScorePixelsCommand extends CommandBase {
    private Elevator elevatorSubsystem;

    public ScorePixelsCommand(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setWheelState(Elevator.WheelState.OUTTAKE);
    }

    @Override
    public boolean isFinished() {
        return !elevatorSubsystem.isBucketFull();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.goToIdlePose();
    }
}
