package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.RangeController;
import org.firstinspires.ftc.teamcode.utils.Rumbler;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.function.BooleanSupplier;

@Config
public class ManualScoringCommand extends CommandBase {
    public static double RELEASE_TIME = 0.6;
    public static double LIFT_FRACTION = 0.8;
    public static double SCORING_INCREMENT = 0.03;
    public static double MAX_SCORING_FRACTION = 1.0;
    public static double MIN_SCORING_FRACTION = 0.5;

    private final ScoringMech scoringMechSubsystem;
    private final BooleanSupplier intakeButton, scoreButton, hangButton, downButton, cancelButton;
    private final Rumbler rumbler;
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private double lastScoringPosition = 0.8;
    private boolean oldUpButton = false;
    private boolean oldDownButton = false;

    private enum ScoringState {
        IDLE,
        INTAKING,
        SCORING,
        RELEASING,
        HANGING,
        RESETTING
    }

    private ScoringState scoringState = ScoringState.INTAKING;

    private final TelemetryHandler telemetryHandler;

    public ManualScoringCommand(ScoringMech scoringMechSubsystem, BooleanSupplier intakeButton,
                                BooleanSupplier scoreButton, BooleanSupplier hangButton,
                                BooleanSupplier downButton, BooleanSupplier cancelButton,
                                Rumbler rumbler, TelemetryHandler telemetryHandler) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.intakeButton = intakeButton;
        this.scoreButton = scoreButton;
        this.hangButton = hangButton;
        this.downButton = downButton;
        this.cancelButton = cancelButton;
        this.rumbler = rumbler;
        this.telemetryHandler = telemetryHandler;

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
                    scoringMechSubsystem.setElevatorHeight(lastScoringPosition);
                    scoringState = ScoringState.SCORING;
                }
                if (hangButton.getAsBoolean()) {
                    scoringMechSubsystem.setElevatorHeight(LIFT_FRACTION);
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.LIFT);
                    scoringState = ScoringState.HANGING;
                }
                break;
            case INTAKING:
                if (!intakeButton.getAsBoolean() || scoringMechSubsystem.getNumPixelsInBucket() == 2) {
                    scoringMechSubsystem.stopIntake();
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoringState = ScoringState.IDLE;
                }
                break;
            case SCORING:
                if (scoringMechSubsystem.getLiftServoState() != ScoringMech.LiftServoState.OUTTAKE && scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.OUTTAKE);
                }

                if (scoreButton.getAsBoolean() && !scoringMechSubsystem.isElevatorBusy()) {
                    oldUpButton = false;
                    oldDownButton = false;

                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.OUTTAKE);
                    scoringState = ScoringState.RELEASING;
                    eTime.reset();
                }

                if (intakeButton.getAsBoolean() && !oldUpButton) {
                    lastScoringPosition += SCORING_INCREMENT;
                }
                if (downButton.getAsBoolean() && !oldDownButton) {
                    lastScoringPosition -= SCORING_INCREMENT;
                }
                lastScoringPosition = RangeController.clamp(lastScoringPosition, MIN_SCORING_FRACTION, MAX_SCORING_FRACTION);
                scoringMechSubsystem.setElevatorHeight(lastScoringPosition);
                oldUpButton = intakeButton.getAsBoolean();
                oldDownButton = downButton.getAsBoolean();

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
            case HANGING:
                if (scoringMechSubsystem.getLiftServoState() != ScoringMech.LiftServoState.LIFT && scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.LIFT);
                }
                if (hangButton.getAsBoolean() && !scoringMechSubsystem.isElevatorBusy()) {
                    scoringMechSubsystem.setElevatorHeight(0);
                    scoringState = ScoringState.RESETTING;
                }
                break;
            case RESETTING:
                if (!scoringMechSubsystem.isElevatorBusy()) {
                    scoringState = ScoringState.IDLE;
                }
                break;
        }

        telemetryHandler.addData("scoring state", scoringState);
    }
}
