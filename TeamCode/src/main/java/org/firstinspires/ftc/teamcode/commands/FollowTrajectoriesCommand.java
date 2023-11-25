package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.List;

public class FollowTrajectoriesCommand extends SequentialCommandGroup {

    public FollowTrajectoriesCommand(DriveSubsystem drive, List<Trajectory> trajectoryList) {
        FollowTrajectoryCommand[] commands = new FollowTrajectoryCommand[trajectoryList.size()];
        for (int i = 0; i < trajectoryList.size(); i++) {
            commands[i] = new FollowTrajectoryCommand(drive, trajectoryList.get(i));
        }
        addCommands(commands);
        addRequirements(drive);
    }
}
