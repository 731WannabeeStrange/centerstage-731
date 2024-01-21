package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class IntakePixelsCommand extends CommandBase {
    private final ScoringMech scoringMechSubsystem;

    public IntakePixelsCommand(ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.startIntake();
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
    }

    @Override
    public boolean isFinished() {
        return scoringMechSubsystem.getNumPixelsInBucket() == 2;
    }

    @Override
    public void end(boolean interrupted) {
        scoringMechSubsystem.stopIntake();
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
    }
}
