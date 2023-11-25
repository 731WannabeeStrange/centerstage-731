package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;

@Config
public class Params {
    // drive model parameters
    public static double inPerTick = 0;
    public static double lateralInPerTick = 1;
    public static double trackWidthTicks = 0;

    // feedforward parameters (in tick units)
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    // path profile parameters (in inches)
    public static double maxWheelVel = 50;
    public static double minProfileAccel = -30;
    public static double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public static double maxAngVel = Math.PI; // shared with path
    public static double maxAngAccel = Math.PI;

    // path controller gains
    public static double axialGain = 0.0;
    public static double lateralGain = 0.0;
    public static double headingGain = 0.0; // shared with turn

    public static double axialVelGain = 0.0;
    public static double lateralVelGain = 0.0;
    public static double headingVelGain = 0.0; // shared with turn

    public static final MecanumKinematics kinematics = new MecanumKinematics(
            inPerTick * trackWidthTicks, inPerTick / lateralInPerTick);

    public static final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            maxAngVel, -maxAngAccel, maxAngAccel);
    public static final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(maxWheelVel),
                    new AngularVelConstraint(maxAngVel)
            ));
    public static final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(minProfileAccel, maxProfileAccel);
}
