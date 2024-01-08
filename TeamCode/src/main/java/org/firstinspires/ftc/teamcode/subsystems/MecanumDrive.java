package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DisplacementTrajectory;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.TrajectoryCommandBuilder;
import org.firstinspires.ftc.teamcode.utils.caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.localization.Localizer;
import org.firstinspires.ftc.teamcode.utils.localization.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.localization.TwoDeadWheelLocalizer;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class MecanumDrive extends SubsystemBase {
    public static Params PARAMS = new Params();
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final VoltageSensor voltageSensor;
    public final IMU imu;
    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private final TelemetryHandler telemetryHandler;
    public Pose2d pose;
    private PoseVelocity2d robotVel;

    public MecanumDrive(HardwareMap hardwareMap, Pose2d startPose, TelemetryHandler telemetryHandler) {
        this.pose = startPose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        leftBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftBack"));
        rightBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBack"));
        rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection,
                PARAMS.usbFacingDirection));
        imu.initialize(parameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, startPose);
        // two-wheel for now so we can have heading without tuned odo
        //localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick, startPose);

        this.telemetryHandler = telemetryHandler;
        register();
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

    @Override
    public void periodic() {
        localizer.update();
        pose = localizer.getPoseEstimate();

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        robotVel = localizer.getPoseVelocity();

        telemetryHandler.addData("x pos", pose.position.x);
        telemetryHandler.addData("y pos", pose.position.y);
        telemetryHandler.addData("heading (deg)", Math.toDegrees(pose.heading.log()));

        // draw current robot pose
        drawRobot(telemetryHandler.fieldOverlay(), pose);

        // draw pose history
        drawPoseHistory(telemetryHandler.fieldOverlay());
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
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

    public class FollowTrajectoryCommand extends CommandBase {
        // for drawing
        private final double[] xPoints, yPoints;
        private final TimeTrajectory trajectory;
        private double beginTime = -1;
        private boolean finished = false;

        public FollowTrajectoryCommand(DisplacementTrajectory trajectory) {
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

            addRequirements(MecanumDrive.this);
        }

        @Override
        public void execute() {
            double currentTime;
            if (beginTime < 0) {
                beginTime = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime());
                currentTime = 0;
            } else {
                currentTime = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime()) - beginTime;
            }

            if (currentTime >= trajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                finished = true;
                return;
            }

            Pose2dDual<Time> txWorldTarget = trajectory.get(currentTime);
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVel);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            // write error to telemetry
            Pose2d error = txWorldTarget.value().minusExp(pose);
            telemetryHandler.addData("x error", error.position.x);
            telemetryHandler.addData("y error", error.position.y);
            telemetryHandler.addData("heading error (deg)", Math.toDegrees(error.heading.log()));

            Canvas c = telemetryHandler.fieldOverlay();

            // draw target pose
            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            // draw trajectory
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }

        @Override
        public boolean isFinished() {
            return finished;
        }
    }

    public class FollowTrajAsPathCommand extends CommandBase {
        // for drawing
        private final double[] xPoints, yPoints;
        private final DisplacementTrajectory trajectory;
        private double disp = 0;
        private boolean finished = false;

        public FollowTrajAsPathCommand(DisplacementTrajectory trajectory) {
            this.trajectory = trajectory;

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

            addRequirements(MecanumDrive.this);
        }

        @Override
        public void execute() {
            if (disp + 1 >= trajectory.length()) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                finished = true;
                return;
            }

            disp = trajectory.project(pose.position, disp);
            Pose2dDual<Time> txWorldTarget = trajectory.get(disp);
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVel);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            // write error to telemetry
            Pose2d error = txWorldTarget.value().minusExp(pose);
            telemetryHandler.addData("x error", error.position.x);
            telemetryHandler.addData("y error", error.position.y);
            telemetryHandler.addData("heading error (deg)", Math.toDegrees(error.heading.log()));

            Canvas c = telemetryHandler.fieldOverlay();

            // draw target pose
            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            // draw trajectory
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }

        @Override
        public boolean isFinished() {
            return finished;
        }
    }

    public class FollowTurnCommand extends CommandBase {
        private final TimeTurn turn;
        private double beginTime = -1;
        private boolean finished = false;

        public FollowTurnCommand(TimeTurn turn) {
            this.turn = turn;
            addRequirements(MecanumDrive.this);
        }

        @Override
        public void execute() {
            double currentTime;
            if (beginTime < 0) {
                beginTime = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime());
                currentTime = 0;
            } else {
                currentTime = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime()) - beginTime;
            }

            if (currentTime >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                finished = true;
                return;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(currentTime);
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVel);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            Canvas c = telemetryHandler.fieldOverlay();

            // draw target pose
            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            // draw turn
            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }

        @Override
        public boolean isFinished() {
            return finished;
        }
    }

    public TrajectoryCommandBuilder commandBuilder(Pose2d beginPose) {
        return new TrajectoryCommandBuilder(
                FollowTurnCommand::new, FollowTrajectoryCommand::new, beginPose,
                1e-6, 0.0, defaultTurnConstraints, defaultVelConstraint,
                defaultAccelConstraint, 0.25, 0.1
        );
    }

    public TrajectoryCommandBuilder pathCommandBuilder(Pose2d beginPose) {
        return new TrajectoryCommandBuilder(
                FollowTurnCommand::new, FollowTrajAsPathCommand::new, beginPose,
                1e-6, 0.0, defaultTurnConstraints, defaultVelConstraint,
                defaultAccelConstraint, 0.25, 0.1
        );
    }

    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        public double inPerTick = 0;
        public double lateralInPerTick = 1;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }
}
