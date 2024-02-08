package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class IntakeFirstCommand extends CommandBase {
    public static double POSITION = 0.75;
    public static double POWER = 0.9;
    public static double TIMEOUT = 1.25;

    private final ScoringMech scoringMechSubsystem;
    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public IntakeFirstCommand(ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setIntakePosition(POSITION);
        scoringMechSubsystem.setIntakePower(POWER);
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.INTAKE);
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished() {
        return scoringMechSubsystem.isFrontColorBlocked() || elapsedTime.time() > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        scoringMechSubsystem.setIntakePosition(ScoringMech.INTAKE_PARAMS.UP_POS);
        scoringMechSubsystem.setIntakePower(0);
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
    }
}
