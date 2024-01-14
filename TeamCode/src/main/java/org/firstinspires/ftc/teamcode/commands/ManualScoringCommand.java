package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.BooleanSupplier;

@Config
public class ManualScoringCommand extends CommandBase {
    private final Intake intakeSubsystem;
    private final Elevator elevatorSubsystem;
    private final BooleanSupplier g1b;
    private final BooleanSupplier g1x;
    private final Runnable rumbler;
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static double RELEASE_FIRST_TIME = 0.75;
    public static double RELEASE_SECOND_TIME = 0.75;

    private enum ScoringState {
        INTAKING,
        BUCKET_FULL_WARNING,
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

    public ManualScoringCommand(Intake intakeSubsystem, Elevator elevatorSubsystem, BooleanSupplier g1b, BooleanSupplier g1x, Runnable rumbler) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.g1b = g1b;
        this.g1x = g1x;
        this.rumbler = rumbler;

        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        switch (scoringState) {
            case IDLE:
                if (g1b.getAsBoolean()) {
                    if (elevatorSubsystem.isBucketFull()) {
                        rumbler.run();
                        scoringState = ScoringState.BUCKET_FULL_WARNING;
                    } else {
                        intakeSubsystem.start();
                        elevatorSubsystem.setWheelState(Elevator.WheelState.INTAKE);
                        scoringState = ScoringState.INTAKING;
                    }
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
            case BUCKET_FULL_WARNING:
                if (!g1b.getAsBoolean()) {
                    scoringState = ScoringState.WAITING_FOR_SCORE;
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
                if (g1x.getAsBoolean()) {
                    elevatorSubsystem.goToIdlePose();
                    scoringState = ScoringState.RESETTING;
                }
                break;
            case SCORING_FIRST:
                if (g1b.getAsBoolean()) {
                    elevatorSubsystem.setWheelState(Elevator.WheelState.OUTTAKE);
                    scoringState = ScoringState.RELEASING_FIRST;
                    eTime.reset();
                }
                if (g1x.getAsBoolean()) {
                    elevatorSubsystem.goToIdlePose();
                    scoringState = ScoringState.RESETTING;
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
