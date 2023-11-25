package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.tuning.Params;

import java.util.List;

public class BuilderUtils {
    public static TrajectoryBuilder getTrajectoryBuilder(Pose2d beginPose) {
        return new TrajectoryBuilder(
                beginPose, 1e-6, 0.0,
                Params.defaultVelConstraint, Params.defaultAccelConstraint,
                0.25, 0.1
        );
    }

    public static TimeTurn turnBuilder(double angle, Pose2d beginPose) {
        return new TimeTurn(
                beginPose, angle, Params.defaultTurnConstraints
        );
    }

    public static TimeTurn turnToBuilder(double heading, Pose2d beginPose) {
        return new TimeTurn(
                beginPose, heading - beginPose.heading.log(), Params.defaultTurnConstraints
        );
    }

    public static Pose2d getEndPose(List<Trajectory> trajectoryList) {
        return trajectoryList.get(trajectoryList.size() - 1).path.end(1).value();
    }

    public static Pose2d getEndPose(TimeTurn turn) {
        return new Pose2d(turn.beginPose.position, turn.beginPose.heading.plus(turn.angle));
    }
}
