package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

public class ResetElevatorCommand extends CommandBase {
    private final ScoringMech scoringMechSubsystem;
    public ResetElevatorCommand(ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;

        //addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.reset();
    }

    @Override
    public boolean isFinished() {
        return !scoringMechSubsystem.isElevatorBusy() && scoringMechSubsystem.getLiftServoState() == ScoringMech.LiftServoState.INTAKE;
    }
}
