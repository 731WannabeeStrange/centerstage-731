package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.BooleanSupplier;

@Config
public class ManualScoringCommand extends CommandBase {
    private Intake intakeSubsystem;
    private Elevator elevatorSubsystem;
    private BooleanSupplier g1b;
    private ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static double RELEASE_FIRST_TIME = 0.75;
    public static double RELEASE_SECOND_TIME = 0.75;

    private enum ScoringState {
        INTAKING,
        WAITING_FOR_SCORE,
        ELEVATING,
        SCORING_FIRST,
        RELEASING_FIRST,
        SCORING_SECOND,
        RELEASING_SECOND,
        RESETTING,
        IDLE
    }
    private ScoringState scoringState = ScoringState.INTAKING;

    public ManualScoringCommand(Intake intakeSubsystem, Elevator elevatorSubsystem, BooleanSupplier g1b) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.g1b = g1b;

        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        switch (scoringState) {
            case IDLE:
                if (g1b.getAsBoolean()) {
                    intakeSubsystem.start();
                    elevatorSubsystem.setWheelState(Elevator.WheelState.INTAKE);
                    scoringState = ScoringState.INTAKING;
                }
                break;
            case INTAKING:
                if (!g1b.getAsBoolean()) {
                    intakeSubsystem.stop();
                    elevatorSubsystem.setWheelState(Elevator.WheelState.STOPPED);
                    scoringState = ScoringState.IDLE;
                }
                if (elevatorSubsystem.isBucketFull()) {
                    intakeSubsystem.stop();
                    elevatorSubsystem.setWheelState(Elevator.WheelState.STOPPED);
                    if (!g1b.getAsBoolean()) {
                        scoringState = ScoringState.WAITING_FOR_SCORE;
                    }
                }
                break;
            case WAITING_FOR_SCORE:
                if (g1b.getAsBoolean()) {
                    elevatorSubsystem.goToMaxScoringPos();
                    scoringState = ScoringState.ELEVATING;
                }
                break;
            case ELEVATING:
                if (elevatorSubsystem.isInScoringPosition()) {
                    scoringState = ScoringState.SCORING_FIRST;
                }
                break;
            case SCORING_FIRST:
                if (g1b.getAsBoolean()) {
                    elevatorSubsystem.setWheelState(Elevator.WheelState.OUTTAKE);
                    scoringState = ScoringState.RELEASING_FIRST;
                    eTime.reset();
                }
                break;
            case RELEASING_FIRST:
                if (eTime.time() > RELEASE_FIRST_TIME) {
                    elevatorSubsystem.setWheelState(Elevator.WheelState.STOPPED);
                    scoringState = ScoringState.SCORING_SECOND;
                }
                break;
            case SCORING_SECOND:
                if (g1b.getAsBoolean()) {
                    elevatorSubsystem.setWheelState(Elevator.WheelState.OUTTAKE);
                    scoringState = ScoringState.RELEASING_SECOND;
                    eTime.reset();
                }
                break;
            case RELEASING_SECOND:
                if (eTime.time() > RELEASE_SECOND_TIME) {
                    elevatorSubsystem.setWheelState(Elevator.WheelState.STOPPED);
                    elevatorSubsystem.goToIdlePose();
                    scoringState = ScoringState.RESETTING;
                }
                break;
            case RESETTING:
                if (elevatorSubsystem.isInIntakePosition()) {
                    scoringState = ScoringState.IDLE;
                }
                break;
        }
    }
}
