package org.firstinspires.ftc.teamcode.utils.localization;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

public interface IncrementalLocalizer {
    Twist2dDual<Time> update();
}
