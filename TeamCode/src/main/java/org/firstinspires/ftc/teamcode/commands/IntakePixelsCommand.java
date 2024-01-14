package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
public class IntakePixelsCommand extends CommandBase {
    private Intake intakeSubsystem;
    private Elevator elevatorSubsystem;

    public IntakePixelsCommand(Intake intakeSubsystem, Elevator elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.start();
        elevatorSubsystem.setWheelState(Elevator.WheelState.INTAKE);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isBucketFull();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        elevatorSubsystem.setWheelState(Elevator.WheelState.STOPPED);
    }
}
