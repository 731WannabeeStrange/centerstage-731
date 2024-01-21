package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class ScorePixelsCommand extends CommandBase {
    private final ScoringMech scoringMechSubsystem;
    private final double fraction;
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum ScoreState {
        GOING_UP,
        RELEASING,
        IDLE
    }
    private ScoreState scoreState = ScoreState.GOING_UP;

    public ScorePixelsCommand(double fraction, ScoringMech scoringMechSubsystem) {
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
        switch (scoreState) {
            case GOING_UP:
                if (scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.OUTTAKE);
                }
                if (scoringMechSubsystem.isElevatorBusy()) {
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.OUTTAKE);
                    eTime.reset();
                    scoreState = ScoreState.RELEASING;
                }
                break;
            case RELEASING:
                if (eTime.time() > 0.6) {
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoreState = ScoreState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return scoreState == ScoreState.IDLE;
    }
}
