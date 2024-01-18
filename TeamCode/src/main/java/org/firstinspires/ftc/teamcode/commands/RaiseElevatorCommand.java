package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@Config
public class RaiseElevatorCommand extends CommandBase {
    private final Elevator elevatorSubsystem;
    private final Elevator.ElevatorState elevatorState;

    public static double SERVO_THRESHOLD = 400;

    public RaiseElevatorCommand(Elevator.ElevatorState elevatorState, Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorState = elevatorState;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorHeight(elevatorState);
    }

    @Override
    public void execute() {
        if (elevatorSubsystem.getElevatorHeight() > SERVO_THRESHOLD) {
            elevatorSubsystem.setLiftServoState(Elevator.LiftServoState.OUTTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return !elevatorSubsystem.isElevatorBusy();
    }
}
