package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

import java.util.function.DoubleSupplier;

public class ManualDriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier g1lx, g1ly, g1rx;
    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();

    public ManualDriveCommand(DriveSubsystem drive, DoubleSupplier g1lx, DoubleSupplier g1ly, DoubleSupplier g1rx) {
        this.drive = drive;
        this.g1lx = g1lx;
        this.g1ly = g1ly;
        this.g1rx = g1rx;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        TelemetryPacket packet = telemetryHandler.getCurrentPacket();

        Vector2d input = drive.getPose().heading.inverse().times(
                new Vector2d(g1ly.getAsDouble(), -g1lx.getAsDouble())
        );
        PoseVelocity2d powers = new PoseVelocity2d(input, -g1rx.getAsDouble());

        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        drive.setDrivePowers(
                wheelVels.leftFront.get(0) / maxPowerMag,
                wheelVels.leftBack.get(0) / maxPowerMag,
                wheelVels.rightBack.get(0) / maxPowerMag,
                wheelVels.rightFront.get(0) / maxPowerMag
        );

        packet.put("left front power", wheelVels.leftFront.get(0) / maxPowerMag);
        packet.put("left back power", wheelVels.leftBack.get(0) / maxPowerMag);
        packet.put("right back power", wheelVels.rightBack.get(0) / maxPowerMag);
        packet.put("right front power", wheelVels.rightFront.get(0) / maxPowerMag);

        telemetryHandler.updateCurrentPacket(packet);
    }
}
