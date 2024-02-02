package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

@Config
public class KnockDownStackCommand extends CommandBase {
    public static double TIME = 0.8;
    public static double INTAKE_POWER = 0.45;

    private final ScoringMech scoringMechSubsystem;
    private final double position;

    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public KnockDownStackCommand(double position, ScoringMech scoringMechSubsystem) {
        this.scoringMechSubsystem = scoringMechSubsystem;
        this.position = position;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setIntakePosition(position);
        scoringMechSubsystem.setIntakePower(INTAKE_POWER);
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.time() > TIME;
    }
}
