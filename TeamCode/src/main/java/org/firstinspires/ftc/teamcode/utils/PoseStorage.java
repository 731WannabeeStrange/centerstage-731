package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = null;

    public static void invalidateStorage() {
        currentPose = null;
    }
}
