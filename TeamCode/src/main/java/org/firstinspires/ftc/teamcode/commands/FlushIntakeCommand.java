package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class FlushIntakeCommand extends CommandBase {
    public static double REVERSE_TIME = 1;

    private final ScoringMech scoringMechSubsystem;
    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public FlushIntakeCommand(ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;

        //addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setIntakePower(-ScoringMech.INTAKE_PARAMS.MOTOR_POWER);
        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.REVERSE_POS);
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.time() > REVERSE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        scoringMechSubsystem.setIntakePower(0);
        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.UP_POS);
    }
}
