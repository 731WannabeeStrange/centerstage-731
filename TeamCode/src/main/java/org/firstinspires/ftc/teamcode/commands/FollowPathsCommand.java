package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.List;

public class FollowPathsCommand extends SequentialCommandGroup {

    public FollowPathsCommand(DriveSubsystem drive, List<Trajectory> trajectoryList) {
        FollowPathCommand[] commands = new FollowPathCommand[trajectoryList.size()];
        for (int i = 0; i < trajectoryList.size(); i++) {
            commands[i] = new FollowPathCommand(drive, trajectoryList.get(i));
        }
        addCommands(commands);
        addRequirements(drive);
    }
}
