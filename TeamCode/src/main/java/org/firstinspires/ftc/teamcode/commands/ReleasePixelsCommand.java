package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;

public class ReleasePixelsCommand extends CommandBase {
    private final ScoringMech scoringMechSubsystem;
    private final double releaseTime;

    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public ReleasePixelsCommand(double releaseTime, ScoringMech scoringMechSubsystem) {
        this.releaseTime = releaseTime;
        this.scoringMechSubsystem = scoringMechSubsystem;

        addRequirements(scoringMechSubsystem);
    }

    @Override
    public void initialize() {
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.OUTTAKE);
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.time() > releaseTime;
    }

    @Override
    public void end(boolean interrupted) {
        scoringMechSubsystem.setWheelState(ScoringMech.WheelState.STOPPED);
    }
}
