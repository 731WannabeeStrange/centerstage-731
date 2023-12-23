package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;

public class FollowPathsCommand extends SequentialCommandGroup {

    public FollowPathsCommand(DriveSubsystem drive, List<Trajectory> trajectoryList, TelemetryHandler telemetryHandler) {
        FollowPathCommand[] commands = new FollowPathCommand[trajectoryList.size()];
        for (int i = 0; i < trajectoryList.size(); i++) {
            commands[i] = new FollowPathCommand(drive, trajectoryList.get(i), telemetryHandler);
        }
        addCommands(commands);
        addRequirements(drive);
    }
}
