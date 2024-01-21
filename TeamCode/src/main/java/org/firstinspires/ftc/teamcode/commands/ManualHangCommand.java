package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

import java.util.function.BooleanSupplier;

public class ManualHangCommand extends CommandBase {
    private ScoringMech scoringMechSubsystem;
    private BooleanSupplier retractButton;

    private enum HangState {
        LIFTING,
        HANGING,
        DROPPING,
        IDLE
    }
    private HangState hangState = HangState.LIFTING;

    public ManualHangCommand(ScoringMech scoringMechSubsystem, BooleanSupplier retractButton) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.retractButton = retractButton;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setElevatorHeight(0.8);
    }

    @Override
    public void execute() {
        switch (hangState) {
            case LIFTING:
                if (scoringMechSubsystem.canLiftServosExtend()) {
                    scoringMechSubsystem.setLiftServoState(ScoringMech.LiftServoState.LIFT);
                }
                if (!scoringMechSubsystem.isElevatorBusy()) {
                    hangState = HangState.HANGING;
                }
                break;
            case HANGING:
                if (retractButton.getAsBoolean()) {
                    scoringMechSubsystem.setElevatorHeight(0.1);
                    hangState = HangState.DROPPING;
                }
                break;
            case DROPPING:
                if (!scoringMechSubsystem.isElevatorBusy()) {
                    hangState = HangState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return hangState == HangState.IDLE;
    }
}
