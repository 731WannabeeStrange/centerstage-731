package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
public class ScorePixelsCommand extends CommandBase {
    private Elevator elevatorSubsystem;
    private Elevator.ElevatorState elevatorState;
    private ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum ScoreState {
        GOING_UP,
        RELEASING,
        IDLE
    }
    private ScoreState scoreState = ScoreState.GOING_UP;

    public ScorePixelsCommand(Elevator elevatorSubsystem, Elevator.ElevatorState elevatorState) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorState = elevatorState;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorHeight(elevatorState);
    }

    @Override
    public void execute() {
        switch (scoreState) {
            case GOING_UP:
                if (elevatorSubsystem.isInScoringPosition()) {
                    elevatorSubsystem.setWheelState(Elevator.WheelState.OUTTAKE);
                    eTime.reset();
                    scoreState = ScoreState.RELEASING;
                }
                break;
            case RELEASING:
                if (eTime.time() > 0.6) {
                    elevatorSubsystem.setWheelState(Elevator.WheelState.STOPPED);
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
