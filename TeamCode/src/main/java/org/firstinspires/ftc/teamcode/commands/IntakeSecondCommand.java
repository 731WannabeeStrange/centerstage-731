package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class IntakeSecondCommand extends CommandBase {
    public static double POSITION = 0.75;
    public static double POWER = 0.9;

    private final ScoringMech scoringMechSubsystem;

    public IntakeSecondCommand(ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setIntakePosition(POSITION);
        scoringMechSubsystem.setIntakePower(POWER);
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
    }

    @Override
    public boolean isFinished() {
        return scoringMechSubsystem.getNumPixelsInBucket() == 2;
    }

    @Override
    public void end(boolean interrupted) {
        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.UP_POS);
        scoringMechSubsystem.setIntakePower(0);
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
    }
}
