package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.ServoPair;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.caching.CachingCRServo;
import org.firstinspires.ftc.teamcode.utils.caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.caching.CachingServo;

@Config
public class ScoringMech extends SubsystemBase {
    public static ElevatorParams ELEVATOR_PARAMS = new ElevatorParams();
    public static IntakeParams INTAKE_PARAMS = new IntakeParams();
    public static LiftServoParams LIFT_SERVO_PARAMS = new LiftServoParams();
    public static BucketParams BUCKET_PARAMS = new BucketParams();

    private final PIDFController rightLiftController = new PIDFController(
            ELEVATOR_PARAMS.kP, ELEVATOR_PARAMS.kI, ELEVATOR_PARAMS.kD, ELEVATOR_PARAMS.kF);
    private final PIDFController leftLiftController = new PIDFController(
            ELEVATOR_PARAMS.kP, ELEVATOR_PARAMS.kI, ELEVATOR_PARAMS.kD, ELEVATOR_PARAMS.kF);
    private final DcMotorEx leftMotor, rightMotor, intakeMotor;
    private final Servo bucketServo;
    private final ServoPair liftServoPair, intakeServoPair;
    private final CRServo wheelServo;
    private final RevColorSensorV3 frontColorSensor, backColorSensor;
    private final TelemetryHandler telemetryHandler;
    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private double currentTarget = ELEVATOR_PARAMS.MIN_POS;
    private LiftServoState liftServoState = LiftServoState.INTAKE;
    private ScoringMechState scoringMechState = ScoringMechState.ACTIVE;

    public ScoringMech(HardwareMap hardwareMap, TelemetryHandler telemetryHandler) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightOuttake");
        intakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        rightLiftController.setTolerance(ELEVATOR_PARAMS.POSITION_TOLERANCE);
        leftLiftController.setTolerance(ELEVATOR_PARAMS.POSITION_TOLERANCE);

        bucketServo = new CachingServo(hardwareMap.get(Servo.class, "bucket"));
        wheelServo = new CachingCRServo(hardwareMap.get(CRServo.class, "wheel"));

        intakeServoPair = new ServoPair(
                new CachingServo(hardwareMap.get(Servo.class, "rightIntake")),
                new CachingServo(hardwareMap.get(Servo.class, "leftIntake"))
        );
        liftServoPair = new ServoPair(
                new CachingServo(hardwareMap.get(Servo.class, "rightLift")),
                new CachingServo(hardwareMap.get(Servo.class, "leftLift"))
        );

        intakeServoPair.setPosition(INTAKE_PARAMS.UP_POS);
        liftServoPair.setPosition(LIFT_SERVO_PARAMS.INTAKE_POS);
        bucketServo.setPosition(BUCKET_PARAMS.INTAKE_POS);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        backColorSensor = hardwareMap.get(RevColorSensorV3.class, "backColor");

        this.telemetryHandler = telemetryHandler;
    }

    @Override
    public void periodic() {
        rightMotor.setPower(rightLiftController.calculate(rightMotor.getCurrentPosition(), currentTarget));
        leftMotor.setPower(leftLiftController.calculate(leftMotor.getCurrentPosition(), currentTarget));

        switch (scoringMechState) {
            case ACTIVE:
                break;
            case WAITING_FOR_TRANSIT_POS:
                if (!areLiftMotorsBusy()) {
                    setLiftServoState(LiftServoState.TRANSIT);
                    scoringMechState = ScoringMechState.WAITING_FOR_TRANSIT_SERVO;
                    elapsedTime.reset();
                }
                break;
            case WAITING_FOR_TRANSIT_SERVO:
                if (elapsedTime.time() > 0.3) {
                    liftServoPair.setPosition(LIFT_SERVO_PARAMS.INTAKE_POS);
                    bucketServo.setPosition((BUCKET_PARAMS.INTAKE_POS + BUCKET_PARAMS.TRANSIT_POS) / 2);
                    scoringMechState = ScoringMechState.WAITING_ON_SERVOS;
                    elapsedTime.reset();
                }
                break;
            case WAIT_FOR_SERVO:
                if (elapsedTime.time() > LIFT_SERVO_PARAMS.OUTTAKE_RESET_TIME) {
                    setElevatorHeight(ELEVATOR_PARAMS.TRANSIT_POS);
                    scoringMechState = ScoringMechState.TRANSIT;
                }
                break;
            case TRANSIT:
                if (!areLiftMotorsBusy()) {
                    liftServoPair.setPosition(LIFT_SERVO_PARAMS.INTAKE_POS);
                    bucketServo.setPosition((BUCKET_PARAMS.INTAKE_POS + BUCKET_PARAMS.TRANSIT_POS) / 2);
                    scoringMechState = ScoringMechState.WAITING_ON_SERVOS;
                    elapsedTime.reset();
                }
                break;
            case WAITING_ON_SERVOS:
                if (elapsedTime.time() > ELEVATOR_PARAMS.SERVO_WAIT_TIME) {
                    setElevatorHeight(ELEVATOR_PARAMS.MIN_POS);
                    scoringMechState = ScoringMechState.RESETTING;
                }
                break;
            case RESETTING:
                if (!areLiftMotorsBusy()) {
                    bucketServo.setPosition(BUCKET_PARAMS.INTAKE_POS);
                    liftServoState = LiftServoState.INTAKE; // for a little housekeeping
                    scoringMechState = ScoringMechState.ACTIVE;
                }
                break;
        }

        telemetryHandler.addData("front color distance (in)", frontColorSensor.getDistance(DistanceUnit.INCH));
        telemetryHandler.addData("back color distance (in)", backColorSensor.getDistance(DistanceUnit.INCH));
        telemetryHandler.addData("num pixels in bucket", getNumPixelsInBucket());
        //telemetryHandler.addData("right outtake position", rightMotor.getCurrentPosition());
        //telemetryHandler.addData("left outtake position", leftMotor.getCurrentPosition());

        //telemetryHandler.addData("right intake servo position", rightServo.getPosition());
        //telemetryHandler.addData("left intake servo position", leftServo.getPosition());
        //telemetryHandler.addData("intake motor power", intakeMotor.getPower());
    }

    private boolean areLiftMotorsBusy() {
        return !rightLiftController.atSetPoint() || !leftLiftController.atSetPoint();
    }

    public void reset() {
        if (getElevatorHeight() < ELEVATOR_PARAMS.TRANSIT_POS) {
            setElevatorHeight(ELEVATOR_PARAMS.TRANSIT_POS + 300);
            scoringMechState = ScoringMechState.WAITING_FOR_TRANSIT_POS;
        } else {
            setLiftServoState(LiftServoState.TRANSIT);
            elapsedTime.reset();
            scoringMechState = ScoringMechState.WAIT_FOR_SERVO;
        }
    }

    public double getElevatorHeight() {
        return Math.abs(rightMotor.getCurrentPosition());
    }

    public void setElevatorHeight(double target) {
        currentTarget = target;
    }

    public void setWheelState(WheelState wheelState) {
        switch (wheelState) {
            case INTAKE:
                wheelServo.setPower(-BUCKET_PARAMS.WHEEL_POWER);
                break;
            case OUTTAKE:
                wheelServo.setPower(BUCKET_PARAMS.WHEEL_POWER);
                break;
            case STOPPED:
                wheelServo.setPower(0);
                break;
        }
    }

    /*
    public void setIntakeState(IntakeState intakeState) {
        switch (intakeState) {
            case STARTED:
                intakeMotor.setPower(INTAKE_PARAMS.MOTOR_POWER);
                intakeServoPair.setPosition(INTAKE_PARAMS.DOWN_POS);
                break;
            case REVERSED:
                intakeMotor.setPower(-INTAKE_PARAMS.MOTOR_POWER);
                intakeServoPair.setPosition(INTAKE_PARAMS.REVERSE_POS);
                break;
            case SLOW_REVERSED:
                intakeMotor.setPower(-INTAKE_PARAMS.OUTTAKE_POWER);
                intakeServoPair.setPosition(INTAKE_PARAMS.REVERSE_POS);
                break;
            case STOPPED:
                intakeMotor.setPower(0);
                intakeServoPair.setPosition(INTAKE_PARAMS.UP_POS);
                break;
        }
    }
     */

    public void setIntakePosition(double position) {
        intakeServoPair.setPosition(position);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public double getFrontColor() {
        return frontColorSensor.getDistance(DistanceUnit.INCH);
    }

    public int getNumPixelsInBucket() {
        boolean frontDetected = frontColorSensor.getDistance(DistanceUnit.INCH) < BUCKET_PARAMS.FRONT_COLOR_THRESHOLD;
        boolean backDetected = backColorSensor.getDistance(DistanceUnit.INCH) < BUCKET_PARAMS.BACK_COLOR_THRESHOLD;
        if (backDetected) {
            if (frontDetected) {
                return 2;
            } else {
                return 1;
            }
        } else {
            if (frontDetected) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    public LiftServoState getLiftServoState() {
        return liftServoState;
    }

    public void setLiftServoState(LiftServoState liftServoState) {
        if (liftServoState != this.liftServoState) {
            switch (liftServoState) {
                case INTAKE:
                    liftServoPair.setPosition(LIFT_SERVO_PARAMS.INTAKE_POS);
                    bucketServo.setPosition(BUCKET_PARAMS.INTAKE_POS);
                    break;
                case TRANSIT:
                    liftServoPair.setPosition(LIFT_SERVO_PARAMS.TRANSIT_POS);
                    bucketServo.setPosition(BUCKET_PARAMS.TRANSIT_POS);
                    break;
                case SCORE_GROUND:
                    liftServoPair.setPosition(LIFT_SERVO_PARAMS.SCORE_GROUND_POS);
                    bucketServo.setPosition(BUCKET_PARAMS.SCORE_GROUND_POS);
                    break;
                case OUTTAKE:
                    liftServoPair.setPosition(LIFT_SERVO_PARAMS.OUTTAKE_POS);
                    bucketServo.setPosition(BUCKET_PARAMS.OUTTAKE_POS);
                    break;
                case LIFT:
                    liftServoPair.setPosition(LIFT_SERVO_PARAMS.HANG_POS);
                    bucketServo.setPosition(BUCKET_PARAMS.HANG_POS);
                    break;
            }
            this.liftServoState = liftServoState;
        }
    }

    public boolean canLiftServosExtend() {
        return getElevatorHeight() > ELEVATOR_PARAMS.SERVO_THRESHOLD;
    }

    public boolean isElevatorBusy() {
        return areLiftMotorsBusy() || scoringMechState != ScoringMechState.ACTIVE;
    }

    public enum WheelState {
        INTAKE,
        OUTTAKE,
        STOPPED
    }

    public enum LiftServoState {
        INTAKE,
        TRANSIT,
        SCORE_GROUND,
        OUTTAKE,
        LIFT
    }

    public enum ScoringMechState {
        ACTIVE,
        WAITING_FOR_TRANSIT_POS,
        WAITING_FOR_TRANSIT_SERVO,
        WAIT_FOR_SERVO,
        TRANSIT,
        WAITING_ON_SERVOS,
        RESETTING
    }

    public static class ElevatorParams {
        public double kP = 0.006;
        public double kI = 0;
        public double kD = 0;
        public double kF = 2e-7;
        public double POSITION_TOLERANCE = 15;
        public double MIN_POS = 25;
        public double MAX_POS = 3000;
        public double SERVO_THRESHOLD = 800;
        public double SERVO_WAIT_TIME = 0.3;
        public double TRANSIT_POS = 500;
    }

    public static class IntakeParams {
        public double MOTOR_POWER = 0.9;
        public double OUTTAKE_POWER = 0.7;
        public double UP_POS = 0.25;
        public double REVERSE_POS = 0.4;
        public double DOWN_POS = 0.75;
    }

    public static class LiftServoParams {
        public double INTAKE_POS = 0.67;
        public double TRANSIT_POS = 0.5;
        public double SCORE_GROUND_POS = 0.68;
        public double OUTTAKE_POS = 0.2;
        public double HANG_POS = 0.2;
        public double OUTTAKE_RESET_TIME = 0.3;
    }

    public static class BucketParams {
        public double INTAKE_POS = 0.3;
        public double TRANSIT_POS = 0.55;
        public double SCORE_GROUND_POS = 0.31;
        public double OUTTAKE_POS = 0.57;
        public double HANG_POS = 0.83;
        public double WHEEL_POWER = 1;
        public double BACK_COLOR_THRESHOLD = 0.5;
        public double FRONT_COLOR_THRESHOLD = 1.15;
    }
}