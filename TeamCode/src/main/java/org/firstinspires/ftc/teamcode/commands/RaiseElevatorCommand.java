package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class RaiseElevatorCommand extends CommandBase {
    private final ScoringMech scoringMechSubsystem;
    private final double fraction;

    public RaiseElevatorCommand(double fraction, ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.fraction = fraction;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setElevatorHeight(fraction);
    }

    @Override
    public void execute() {
        if (scoringMechSubsystem.canLiftServosExtend()) {
            scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.OUTTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return !scoringMechSubsystem.isElevatorBusy();
    }
}
