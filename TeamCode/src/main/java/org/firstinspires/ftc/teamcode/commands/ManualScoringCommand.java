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
    public static double RELEASE_TIME = 0.3;
    public static double LIFT_UP_POS = 2000;
    public static double LIFT_DOWN_POS = 150;
    public static double SCORING_INCREMENT = 150;
    public static double MIN_SCORING_POS = 1000;

    private final ScoringMech scoringMechSubsystem;
    private final BooleanSupplier intakeButton, reverseIntakeButton, scoreButton, hangButton, downButton, cancelButton;
    private final Rumbler rumbler;
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final TelemetryHandler telemetryHandler;

    private double lastScoringPosition = 1600;
    private boolean oldUpButton = false;
    private boolean oldDownButton = false;
    private ScoringState scoringState = ScoringState.INTAKING;

    public ManualScoringCommand(ScoringMech scoringMechSubsystem, BooleanSupplier intakeButton,
                                BooleanSupplier reverseIntakeButton, BooleanSupplier scoreButton,
                                BooleanSupplier hangButton, BooleanSupplier downButton,
                                BooleanSupplier cancelButton, Rumbler rumbler,
                                TelemetryHandler telemetryHandler) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.intakeButton = intakeButton;
        this.reverseIntakeButton = reverseIntakeButton;
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
                    if (reverseIntakeButton.getAsBoolean()) {
                        scoringMechSubsystem.setIntakeState(ScoringMech.IntakeState.REVERSED);
                        scoringState = ScoringState.REVERSE_INTAKE;
                    } else if (scoringMechSubsystem.getNumPixelsInBucket() == 2) {
                        rumbler.rumble(500);
                    } else {
                        scoringMechSubsystem.setIntakeState(ScoringMech.IntakeState.STARTED);
                        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
                        scoringState = ScoringState.INTAKING;
                    }
                }
                if (scoreButton.getAsBoolean() && scoringMechSubsystem.getNumPixelsInBucket() > 0) {
                    scoringMechSubsystem.setElevatorHeight(lastScoringPosition);
                    scoringState = ScoringState.ELEVATE_TO_SCORE;
                }
                if (hangButton.getAsBoolean()) {
                    scoringMechSubsystem.setElevatorHeight(LIFT_UP_POS);
                    scoringState = ScoringState.ELEVATE_TO_HANG;
                }
                break;
            case INTAKING:
                if (!intakeButton.getAsBoolean() || scoringMechSubsystem.getNumPixelsInBucket() == 2) {
                    scoringMechSubsystem.setIntakeState(ScoringMech.IntakeState.STOPPED);
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoringState = ScoringState.IDLE;
                } else if (reverseIntakeButton.getAsBoolean()) {
                    scoringMechSubsystem.setIntakeState(ScoringMech.IntakeState.REVERSED);
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoringState = ScoringState.REVERSE_INTAKE;
                }
                break;
            case REVERSE_INTAKE:
                if (!intakeButton.getAsBoolean()) {
                    scoringMechSubsystem.setIntakeState(ScoringMech.IntakeState.STOPPED);
                    scoringState = ScoringState.IDLE;
                } else if (!reverseIntakeButton.getAsBoolean()) {
                    scoringMechSubsystem.setIntakeState(ScoringMech.IntakeState.STARTED);
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
                    scoringState = ScoringState.INTAKING;
                }
                break;
            case ELEVATE_TO_SCORE:
                if (!scoreButton.getAsBoolean()) {
                    scoringState = ScoringState.SCORING;
                }
                break;
            case SCORING:
                if (scoringMechSubsystem.canLiftServosExtend()) {
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
                lastScoringPosition = RangeController.clamp(lastScoringPosition, MIN_SCORING_POS, ScoringMech.ELEVATOR_PARAMS.MAX_POS);
                scoringMechSubsystem.setElevatorHeight(lastScoringPosition);
                oldUpButton = intakeButton.getAsBoolean();
                oldDownButton = downButton.getAsBoolean();

                if (cancelButton.getAsBoolean()) {
                    scoringMechSubsystem.reset();
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
                        scoringMechSubsystem.reset();
                        scoringState = ScoringState.RESETTING;
                    }
                }
                break;
            case ELEVATE_TO_HANG:
                if (!hangButton.getAsBoolean()) {
                    scoringState = ScoringState.WAITING_FOR_HANG;
                }
                break;
            case WAITING_FOR_HANG:
                if (scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.LIFT);
                }
                if (hangButton.getAsBoolean() && !scoringMechSubsystem.isElevatorBusy()) {
                    scoringMechSubsystem.setElevatorHeight(LIFT_DOWN_POS);
                    scoringState = ScoringState.HANGING;
                }
                if (cancelButton.getAsBoolean()) {
                    scoringMechSubsystem.reset();
                    scoringState = ScoringState.RESETTING;
                }
                break;
            case HANGING:
                if (cancelButton.getAsBoolean()) {
                    scoringMechSubsystem.reset();
                    scoringState = ScoringState.RESETTING;
                }
                break;
            case RESETTING:
                if (!scoringMechSubsystem.isElevatorBusy() && scoringMechSubsystem.getLiftServoState() == ScoringMech.LiftServoState.INTAKE) {
                    scoringState = ScoringState.IDLE;
                }
                break;
        }

        telemetryHandler.addData("scoring state", scoringState);
    }

    private enum ScoringState {
        IDLE,
        INTAKING,
        REVERSE_INTAKE,
        ELEVATE_TO_SCORE,
        SCORING,
        RELEASING,
        ELEVATE_TO_HANG,
        WAITING_FOR_HANG,
        HANGING,
        RESETTING
    }
}
