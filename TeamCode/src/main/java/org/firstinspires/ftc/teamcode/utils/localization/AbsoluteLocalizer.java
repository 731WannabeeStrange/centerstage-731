package org.firstinspires.ftc.teamcode.utils.localization;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.Optional;

public interface AbsoluteLocalizer {
    Optional<Pose2d> getPoseEstimate();
}