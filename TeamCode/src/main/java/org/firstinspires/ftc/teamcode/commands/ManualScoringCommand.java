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
    public static double RELEASE_TIME = 0.4;
    public static double WAIT_COLOR_SENSOR_TIME = 0.3;
    public static double LIFT_UP_POS = 2000;
    public static double LIFT_DOWN_POS = 150;
    public static double SCORING_INCREMENT = 300;
    public static double MIN_SCORING_POS = 1000;

    private final ScoringMech scoringMechSubsystem;
    private final BooleanSupplier intakeButton, reverseIntakeButton, scoreButton, hangButton, upButton, downButton, cancelButton;
    private final Rumbler rumbler;
    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final TelemetryHandler telemetryHandler;

    private double lastScoringPosition = 2400;
    private boolean oldUpButton = false, oldDownButton = false, oldScoreButton = false, oldHangButton = false;
    private ScoringState scoringState = ScoringState.INTAKING;

    public ManualScoringCommand(ScoringMech scoringMechSubsystem, BooleanSupplier intakeButton,
                                BooleanSupplier reverseIntakeButton, BooleanSupplier scoreButton,
                                BooleanSupplier hangButton, BooleanSupplier upButton,
                                BooleanSupplier downButton, BooleanSupplier cancelButton,
                                Rumbler rumbler, TelemetryHandler telemetryHandler) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.intakeButton = intakeButton;
        this.reverseIntakeButton = reverseIntakeButton;
        this.scoreButton = scoreButton;
        this.hangButton = hangButton;
        this.upButton = upButton;
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
                        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.REVERSE_POS);
                        scoringMechSubsystem.setIntakePower(-ScoringMech.INTAKE_PARAMS.MOTOR_POWER);
                        scoringState = ScoringState.REVERSE_INTAKE;
                    } else if (scoringMechSubsystem.getNumPixelsInBucket() == 2) {
                        rumbler.rumble(500);
                    } else {
                        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.DOWN_POS);
                        scoringMechSubsystem.setIntakePower(ScoringMech.INTAKE_PARAMS.MOTOR_POWER);
                        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
                        scoringState = ScoringState.INTAKING;
                    }
                }
                if (scoreButton.getAsBoolean() && scoringMechSubsystem.getNumPixelsInBucket() > 0) {
                    scoringMechSubsystem.setElevatorHeight(lastScoringPosition);
                    scoringState = ScoringState.SCORING;
                }
                if (hangButton.getAsBoolean()) {
                    scoringMechSubsystem.setElevatorHeight(LIFT_UP_POS);
                    scoringState = ScoringState.HANGING;
                }
                break;
            case INTAKING:
                if (!intakeButton.getAsBoolean() || scoringMechSubsystem.getNumPixelsInBucket() == 2) {
                    scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.UP_POS);
                    scoringMechSubsystem.setIntakePower(0);
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoringState = ScoringState.IDLE;
                } else if (reverseIntakeButton.getAsBoolean()) {
                    scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.REVERSE_POS);
                    scoringMechSubsystem.setIntakePower(-ScoringMech.INTAKE_PARAMS.MOTOR_POWER);
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoringState = ScoringState.REVERSE_INTAKE;
                }
                break;
            case REVERSE_INTAKE:
                if (!intakeButton.getAsBoolean()) {
                    scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.UP_POS);
                    scoringMechSubsystem.setIntakePower(0);
                    scoringState = ScoringState.IDLE;
                } else if (!reverseIntakeButton.getAsBoolean()) {
                    scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.DOWN_POS);
                    scoringMechSubsystem.setIntakePower(ScoringMech.INTAKE_PARAMS.MOTOR_POWER);
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
                    scoringState = ScoringState.INTAKING;
                }
                break;
            case SCORING:
                if (scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.OUTTAKE);
                }

                if (scoreButton.getAsBoolean() && !oldScoreButton && !scoringMechSubsystem.isElevatorBusy()) {
                    oldUpButton = false;
                    oldDownButton = false;

                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.OUTTAKE);
                    scoringState = ScoringState.RELEASING;
                    eTime.reset();
                }

                if (upButton.getAsBoolean() && !oldUpButton) {
                    lastScoringPosition += SCORING_INCREMENT;
                }
                if (downButton.getAsBoolean() && !oldDownButton) {
                    lastScoringPosition -= SCORING_INCREMENT;
                }
                lastScoringPosition = RangeController.clamp(lastScoringPosition, MIN_SCORING_POS, ScoringMech.ELEVATOR_PARAMS.MAX_POS);
                scoringMechSubsystem.setElevatorHeight(lastScoringPosition);

                if (cancelButton.getAsBoolean()) {
                    scoringMechSubsystem.reset();
                    scoringState = ScoringState.RESETTING;
                }

                telemetryHandler.addData("last scoring pos", lastScoringPosition);

                break;
            case RELEASING:
                if (eTime.time() > RELEASE_TIME) {
                    scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
                    scoringState = ScoringState.WAIT_FOR_COLOR_SENSOR;
                    eTime.reset();
                }
                break;
            case WAIT_FOR_COLOR_SENSOR:
                if (eTime.time() > WAIT_COLOR_SENSOR_TIME) {
                    if (scoringMechSubsystem.getNumPixelsInBucket() > 0) {
                        scoringState = ScoringState.SCORING;
                    } else {
                        scoringMechSubsystem.reset();
                        scoringState = ScoringState.RESETTING;
                    }
                }
                break;
            case WAITING_FOR_HANG:
                if (scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.LIFT);
                }
                if (hangButton.getAsBoolean() && !oldHangButton && !scoringMechSubsystem.isElevatorBusy()) {
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

        oldUpButton = upButton.getAsBoolean();
        oldDownButton = downButton.getAsBoolean();
        oldHangButton = hangButton.getAsBoolean();
        oldScoreButton = scoreButton.getAsBoolean();

        telemetryHandler.addData("scoring state", scoringState);
    }

    private enum ScoringState {
        IDLE,
        INTAKING,
        REVERSE_INTAKE,
        SCORING,
        RELEASING,
        WAIT_FOR_COLOR_SENSOR,
        WAITING_FOR_HANG,
        HANGING,
        RESETTING
    }
}
