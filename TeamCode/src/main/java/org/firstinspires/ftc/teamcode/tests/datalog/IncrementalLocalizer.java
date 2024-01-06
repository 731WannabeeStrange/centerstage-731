package org.firstinspires.ftc.teamcode.tests.datalog;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

public interface IncrementalLocalizer {
    Twist2dDual<Time> update();
}
