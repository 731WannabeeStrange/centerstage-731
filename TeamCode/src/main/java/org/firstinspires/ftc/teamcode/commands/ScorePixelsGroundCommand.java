package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class ScorePixelsGroundCommand extends CommandBase {
    public static double INTAKE_RELEASE_TIME = 0.25;

    private final ScoringMech scoringMechSubsystem;
    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public ScorePixelsGroundCommand(ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.SCORE_GROUND);
        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.REVERSE_POS);
        scoringMechSubsystem.setIntakePower(-ScoringMech.INTAKE_PARAMS.OUTTAKE_POWER);
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.time() > INTAKE_RELEASE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.UP_POS);
        scoringMechSubsystem.setIntakePower(0);
        scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.INTAKE);
    }
}
