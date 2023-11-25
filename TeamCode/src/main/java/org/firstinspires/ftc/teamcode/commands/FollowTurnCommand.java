package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTurn;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.tuning.Params;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.concurrent.TimeUnit;

public class FollowTurnCommand extends CommandBase {
    private final HolonomicController holonomicController = new HolonomicController(
            Params.axialGain, Params.lateralGain, Params.headingGain,
            Params.axialVelGain, Params.lateralVelGain, Params.headingVelGain
    );
    private double beginTime = -1;
    private boolean finished = false;

    private final DriveSubsystem drive;
    private final TimeTurn turn;

    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();

    public FollowTurnCommand(DriveSubsystem drive, TimeTurn turn) {
        this.drive = drive;
        this.turn = turn;
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

        if (currentTime >= turn.duration) {
            drive.setDrivePowers(0, 0, 0, 0);

            finished = true;
            return;
        }

        Pose2dDual<Time> txWorldTarget = turn.get(currentTime);

        PoseVelocity2dDual<Time> command = holonomicController.compute(txWorldTarget, drive.pose, drive.robotVel);
        MecanumKinematics.WheelVelocities<Time> wheelVels = Params.kinematics.inverse(command);
        double voltage = drive.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(Params.kS, Params.kV / Params.inPerTick, Params.kA / Params.inPerTick);
        drive.setDrivePowers(
                feedforward.compute(wheelVels.leftFront) / voltage,
                feedforward.compute(wheelVels.leftBack) / voltage,
                feedforward.compute(wheelVels.rightBack) / voltage,
                feedforward.compute(wheelVels.rightFront) / voltage
        );

        Canvas c = packet.fieldOverlay();

        // draw target pose
        c.setStroke("#4CAF50");
        DriveSubsystem.drawRobot(c, txWorldTarget.value());

        // draw turn
        c.setStroke("#7C4DFFFF");
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

        telemetryHandler.updateCurrentPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
