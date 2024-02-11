package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class ScorePixelsGroundCommand extends CommandBase {
    public static double BUCKET_RELEASE_TIME = 0.4;
    public static double INTAKE_RELEASE_TIME = 0.5;

    private final ScoringMech scoringMechSubsystem;
    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum ScoreGroundState {
        BUCKET,
        INTAKE,
        IDLE
    }
    // change back to BUCKET to get wheel to spin
    private ScoreGroundState scoreGroundState = ScoreGroundState.INTAKE;

    public ScorePixelsGroundCommand(ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.SCORE_GROUND);
        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.REVERSE_POS);
        scoringMechSubsystem.setIntakePower(-ScoringMech.INTAKE_PARAMS.OUTTAKE_POWER);
        //scoringMechSubsystem.setWheelState(ScoringMech.WheelState.OUTTAKE);
        elapsedTime.reset();
    }

    @Override
    public void execute() {
        switch (scoreGroundState) {
            case BUCKET:
                if (elapsedTime.time() > BUCKET_RELEASE_TIME) {
                    //scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    elapsedTime.reset();
                    scoreGroundState = ScoreGroundState.INTAKE;
                }
                break;
            case INTAKE:
                if (elapsedTime.time() > INTAKE_RELEASE_TIME) {
                    scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.UP_POS);
                    scoringMechSubsystem.setIntakePower(0);
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.INTAKE);
                    scoreGroundState = ScoreGroundState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return scoreGroundState == ScoreGroundState.IDLE;
    }
}
