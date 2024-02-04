package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class ScorePixelsCommand extends CommandBase {
    public static double RELEASE_TIME = 0.55;
    public static double WAIT_TIME = 0.1;

    private final ScoringMech scoringMechSubsystem;
    private final double height;
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum ScoreState {
        GOING_UP,
        RELEASING,
        WAITING,
        IDLE
    }
    private ScoreState scoreState = ScoreState.GOING_UP;

    public ScorePixelsCommand(double height, ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.height = height;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setElevatorHeight(height);
    }

    @Override
    public void execute() {
        switch (scoreState) {
            case GOING_UP:
                if (scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.OUTTAKE);
                }
                if (!scoringMechSubsystem.isElevatorBusy()) {
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.OUTTAKE);
                    eTime.reset();
                    scoreState = ScoreState.RELEASING;
                }
                break;
            case RELEASING:
                if (eTime.time() > RELEASE_TIME) {
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    eTime.reset();
                    scoreState = ScoreState.WAITING;
                }
                break;
            case WAITING:
                if (eTime.time() > WAIT_TIME) {
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
