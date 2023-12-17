package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.tuning.Params;
import org.firstinspires.ftc.teamcode.utils.Localizer;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.TwoDeadWheelLocalizer;

import java.util.LinkedList;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    private final VoltageSensor voltageSensor;

    private final IMU imu;

    private final Localizer localizer;
    private Pose2d pose;
    private PoseVelocity2d robotVel;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();

    public DriveSubsystem(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "flMotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "rlMotor");
        rightBack = hardwareMap.get(DcMotorEx.class, "rrMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frMotor");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // localizer = new ThreeDeadWheelLocalizer(hardwareMap, Params.inPerTick);
        // two-wheel for now so we can have heading without tuned odo
        localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, Params.inPerTick);

        register();
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = telemetryHandler.getCurrentPacket();

        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        robotVel = twist.velocity().value();

        packet.put("x pos", pose.position.x);
        packet.put("y pos", pose.position.y);
        packet.put("heading (deg)", Math.toDegrees(pose.heading.log()));

        // draw current robot pose
        drawRobot(packet.fieldOverlay(), pose);

        // draw pose history
        drawPoseHistory(packet.fieldOverlay());

        telemetryHandler.updateCurrentPacket(packet);
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public void setDrivePowers(double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public Pose2d getPose() {
        return pose;
    }

    public PoseVelocity2d getRobotVel() {
        return robotVel;
    }
}
