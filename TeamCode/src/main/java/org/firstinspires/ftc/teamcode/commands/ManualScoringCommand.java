package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.Rumbler;

import java.util.function.BooleanSupplier;

@Config
public class ManualScoringCommand extends CommandBase {
    private final ScoringMech scoringMechSubsystem;
    private final BooleanSupplier intakeButton, scoreButton, cancelButton;
    private final Rumbler rumbler;
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static double RELEASE_TIME = 0.6;

    private enum ScoringState {
        IDLE,
        INTAKING,
        ELEVATING,
        SCORING,
        RELEASING,
        RESETTING
    }
    private ScoringState scoringState = ScoringState.INTAKING;

    public ManualScoringCommand(ScoringMech scoringMechSubsystem, BooleanSupplier intakeButton,
                                BooleanSupplier scoreButton, BooleanSupplier cancelButton,
                                Rumbler rumbler) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.intakeButton = intakeButton;
        this.scoreButton = scoreButton;
        this.cancelButton = cancelButton;
        this.rumbler = rumbler;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void execute() {
        switch (scoringState) {
            case IDLE:
                if (intakeButton.getAsBoolean()) {
                    if (scoringMechSubsystem.getNumPixelsInBucket() == 2) {
                        rumbler.rumble(500);
                    } else {
                        scoringMechSubsystem.startIntake();
                        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
                        scoringState = ScoringState.INTAKING;
                    }
                }
                if (scoreButton.getAsBoolean() && scoringMechSubsystem.getNumPixelsInBucket() > 0) {
                    scoringMechSubsystem.setElevatorHeight(0.8);
                    scoringState = ScoringState.ELEVATING;
                }
                break;
            case INTAKING:
                if (!intakeButton.getAsBoolean() || scoringMechSubsystem.getNumPixelsInBucket() == 2) {
                    scoringMechSubsystem.stopIntake();
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoringState = ScoringState.IDLE;
                }
                break;
            case ELEVATING:
                if (!scoringMechSubsystem.isElevatorBusy()) {
                    scoringState = ScoringState.SCORING;
                }
                if (cancelButton.getAsBoolean()) {
                    scoringMechSubsystem.setElevatorHeight(0);
                    scoringState = ScoringState.RESETTING;
                }
                break;
            case SCORING:
                if (scoreButton.getAsBoolean()) {
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.OUTTAKE);
                    scoringState = ScoringState.RELEASING;
                    eTime.reset();
                }
                if (cancelButton.getAsBoolean()) {
                    scoringMechSubsystem.setElevatorHeight(0);
                    scoringState = ScoringState.RESETTING;
                }
                break;
            case RELEASING:
                if (eTime.time() > RELEASE_TIME) {
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    if (scoringMechSubsystem.getNumPixelsInBucket() > 0) {
                        scoringState = ScoringState.SCORING;
                    } else {
                        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                        scoringMechSubsystem.setElevatorHeight(0);
                        scoringState = ScoringState.RESETTING;
                    }
                }
                break;
            case RESETTING:
                if (!scoringMechSubsystem.isElevatorBusy()) {
                    scoringState = ScoringState.IDLE;
                }
                break;
        }
    }
}
