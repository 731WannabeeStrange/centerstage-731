package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.tuning.Params;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class FollowTrajectoryCommand extends CommandBase {
    private final HolonomicController holonomicController = new HolonomicController(
            Params.axialGain, Params.lateralGain, Params.headingGain,
            Params.axialVelGain, Params.lateralVelGain, Params.headingVelGain
    );
    private double beginTime = -1;
    private boolean finished = false;

    // for drawing
    private final double[] xPoints, yPoints;

    private final DriveSubsystem drive;
    private final TimeTrajectory trajectory;

    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();

    public FollowTrajectoryCommand(DriveSubsystem drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = new TimeTrajectory(trajectory);

        List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                0, this.trajectory.path.length(),
                Math.max(2, (int) Math.ceil(this.trajectory.path.length() / 2)));
        xPoints = new double[disps.size()];
        yPoints = new double[disps.size()];
        for (int i = 0; i < disps.size(); i++) {
            Pose2d p = this.trajectory.path.get(disps.get(i), 1).value();
            xPoints[i] = p.position.x;
            yPoints[i] = p.position.y;
        }

        addRequirements(drive);
    }

    @Override
    public void execute() {
        TelemetryPacket packet = telemetryHandler.getCurrentPacket();

        double currentTime;
        if (beginTime < 0) {
            beginTime = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime());
            currentTime = 0;
        } else {
            currentTime = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime()) - beginTime;
        }

        if (currentTime >= trajectory.duration) {
            drive.setDrivePowers(0, 0, 0, 0);

            finished = true;
            return;
        }

        Pose2dDual<Time> txWorldTarget = trajectory.get(currentTime);

        PoseVelocity2dDual<Time> command = holonomicController.compute(txWorldTarget, drive.getPose(), drive.getRobotVel());
        MecanumKinematics.WheelVelocities<Time> wheelVels = Params.kinematics.inverse(command);
        double voltage = drive.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(Params.kS, Params.kV / Params.inPerTick, Params.kA / Params.inPerTick);
        drive.setDrivePowers(
                feedforward.compute(wheelVels.leftFront) / voltage,
                feedforward.compute(wheelVels.leftBack) / voltage,
                feedforward.compute(wheelVels.rightBack) / voltage,
                feedforward.compute(wheelVels.rightFront) / voltage
        );

        // write error to telemetry
        Pose2d error = txWorldTarget.value().minusExp(drive.getPose());
        packet.put("x error", error.position.x);
        packet.put("y error", error.position.y);
        packet.put("heading error (deg)", Math.toDegrees(error.heading.log()));

        Canvas c = packet.fieldOverlay();

        // draw target pose
        c.setStroke("#4CAF50");
        DriveSubsystem.drawRobot(c, txWorldTarget.value());

        // draw trajectory
        c.setStroke("#4CAF507A");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);

        telemetryHandler.updateCurrentPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
