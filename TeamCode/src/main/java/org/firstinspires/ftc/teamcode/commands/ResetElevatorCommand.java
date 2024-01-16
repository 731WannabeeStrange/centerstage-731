package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ResetElevatorCommand extends CommandBase {
    private Elevator elevatorSubsystem;
    public ResetElevatorCommand(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorHeight(Elevator.ElevatorState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isInIntakePosition();
    }
}
