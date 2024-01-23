package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.caching.CachingCRServo;
import org.firstinspires.ftc.teamcode.utils.caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.caching.CachingServo;

@Config
public class ScoringMech extends SubsystemBase {
    public static TrapezoidProfile.Constraints liftConstraints = new TrapezoidProfile.Constraints(1500, 1500);
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0.005, 0, 0);
    public static ElevatorParams ELEVATOR_PARAMS = new ElevatorParams();
    public static IntakeParams INTAKE_PARAMS = new IntakeParams();
    public static LiftServoParams LIFT_SERVO_PARAMS = new LiftServoParams();
    public static BucketParams BUCKET_PARAMS = new BucketParams();

    private final ProfiledPIDController liftController = new ProfiledPIDController(
            liftCoefficients.p,
            liftCoefficients.i,
            liftCoefficients.d,
            liftConstraints
    );
    private final DcMotorEx leftMotor, rightMotor, intakeMotor;
    private final Servo rightIntakeServo, leftIntakeServo, bucketServo, leftLiftServo, rightLiftServo;
    private final CRServo wheelServo;
    private final RevColorSensorV3 frontColorSensor, backColorSensor;
    private final TelemetryHandler telemetryHandler;
    private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private double currentTarget = ELEVATOR_PARAMS.MIN_POS;
    private LiftServoState liftServoState = LiftServoState.INTAKE;
    private ScoringMechState scoringMechState = ScoringMechState.ACTIVE;

    public ScoringMech(HardwareMap hardwareMap, TelemetryHandler telemetryHandler) {
        liftController.setTolerance(20);

        leftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftOuttake"));
        rightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightOuttake"));
        intakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        bucketServo = new CachingServo(hardwareMap.get(Servo.class, "bucket"));
        leftLiftServo = new CachingServo(hardwareMap.get(Servo.class, "leftLift"));
        rightLiftServo = new CachingServo(hardwareMap.get(Servo.class, "rightLift"));
        rightIntakeServo = new CachingServo(hardwareMap.get(Servo.class, "rightIntake"));
        leftIntakeServo = new CachingServo(hardwareMap.get(Servo.class, "leftIntake"));
        wheelServo = new CachingCRServo(hardwareMap.get(CRServo.class, "wheel"));

        rightLiftServo.setPosition(LIFT_SERVO_PARAMS.INTAKE_RIGHT_POSITION);
        leftLiftServo.setPosition(LIFT_SERVO_PARAMS.INTAKE_LEFT_POSITION);
        bucketServo.setPosition(BUCKET_PARAMS.INTAKE_POSITION);
        rightIntakeServo.setPosition(INTAKE_PARAMS.RIGHT_UP_POS);
        leftIntakeServo.setPosition(INTAKE_PARAMS.LEFT_UP_POS);
        leftIntakeServo.setDirection(Servo.Direction.REVERSE);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        backColorSensor = hardwareMap.get(RevColorSensorV3.class, "backColor");

        this.telemetryHandler = telemetryHandler;
    }

    @Override
    public void periodic() {
        switch (scoringMechState) {
            case ACTIVE:
                rightMotor.setPower(liftController.calculate(rightMotor.getCurrentPosition(), currentTarget));
                leftMotor.setPower(liftController.calculate(leftMotor.getCurrentPosition(), currentTarget));
                if (Math.abs(rightMotor.getCurrentPosition()) < ELEVATOR_PARAMS.TRANSIT_THRESHOLD) {
                    setLiftServoState(LiftServoState.INTAKE);
                }
                break;
            case WAITING_ON_SERVOS:
                if (elapsedTime.time() > ELEVATOR_PARAMS.SERVO_WAIT_TIME) {
                    // set motor powers here to avoid funny stuff with the isBusy
                    rightMotor.setPower(liftController.calculate(rightMotor.getCurrentPosition(), currentTarget));
                    leftMotor.setPower(liftController.calculate(leftMotor.getCurrentPosition(), currentTarget));

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

    public void reset() {
        setLiftServoState(LiftServoState.TRANSIT);
        currentTarget = ELEVATOR_PARAMS.MIN_POS;
        if (getElevatorHeight() < ELEVATOR_PARAMS.SERVO_WAIT_THRESHOLD) {
            scoringMechState = ScoringMechState.WAITING_ON_SERVOS;
            elapsedTime.reset();
        } else {
            scoringMechState = ScoringMechState.ACTIVE;
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

    public void setIntakeState(IntakeState intakeState) {
        switch (intakeState) {
            case STARTED:
                intakeMotor.setPower(INTAKE_PARAMS.MOTOR_POWER);
                rightIntakeServo.setPosition(INTAKE_PARAMS.RIGHT_UP_POS + INTAKE_PARAMS.DOWN_OFFSET);
                leftIntakeServo.setPosition(INTAKE_PARAMS.LEFT_UP_POS + INTAKE_PARAMS.DOWN_OFFSET);
                break;
            case REVERSED:
                intakeMotor.setPower(-INTAKE_PARAMS.MOTOR_POWER);
                rightIntakeServo.setPosition(INTAKE_PARAMS.RIGHT_UP_POS + INTAKE_PARAMS.REVERSE_OFFSET);
                leftIntakeServo.setPosition(INTAKE_PARAMS.LEFT_UP_POS + INTAKE_PARAMS.REVERSE_OFFSET);
                break;
            case STOPPED:
                intakeMotor.setPower(0);
                rightIntakeServo.setPosition(INTAKE_PARAMS.RIGHT_UP_POS);
                leftIntakeServo.setPosition(INTAKE_PARAMS.LEFT_UP_POS);
                break;
        }
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
                    rightLiftServo.setPosition(LIFT_SERVO_PARAMS.INTAKE_RIGHT_POSITION);
                    leftLiftServo.setPosition(LIFT_SERVO_PARAMS.INTAKE_LEFT_POSITION);
                    bucketServo.setPosition(BUCKET_PARAMS.INTAKE_POSITION);
                    break;
                case TRANSIT:
                    rightLiftServo.setPosition(LIFT_SERVO_PARAMS.INTAKE_RIGHT_POSITION + LIFT_SERVO_PARAMS.TRANSIT_OFFSET);
                    leftLiftServo.setPosition(LIFT_SERVO_PARAMS.INTAKE_LEFT_POSITION - LIFT_SERVO_PARAMS.TRANSIT_OFFSET);
                    bucketServo.setPosition(BUCKET_PARAMS.INTAKE_POSITION);
                    break;
                case OUTTAKE:
                    rightLiftServo.setPosition(LIFT_SERVO_PARAMS.OUTTAKE_RIGHT_POSITION);
                    leftLiftServo.setPosition(LIFT_SERVO_PARAMS.OUTTAKE_LEFT_POSITION);
                    bucketServo.setPosition(BUCKET_PARAMS.OUTTAKE_POSITION);
                    break;
                case LIFT:
                    rightLiftServo.setPosition(LIFT_SERVO_PARAMS.HANG_RIGHT_POSITION);
                    leftLiftServo.setPosition(LIFT_SERVO_PARAMS.HANG_LEFT_POSITION);
                    bucketServo.setPosition(BUCKET_PARAMS.HANG_POSITION);
                    break;
            }
            this.liftServoState = liftServoState;
        }
    }

    public boolean canLiftServosExtend() {
        return getElevatorHeight() > ELEVATOR_PARAMS.SERVO_THRESHOLD;
    }

    public boolean isElevatorBusy() {
        return !liftController.atGoal() || scoringMechState == ScoringMechState.WAITING_ON_SERVOS;
    }

    public enum IntakeState {
        STARTED,
        REVERSED,
        STOPPED
    }

    public enum WheelState {
        INTAKE,
        OUTTAKE,
        STOPPED
    }

    public enum LiftServoState {
        INTAKE,
        TRANSIT,
        OUTTAKE,
        LIFT
    }

    public enum ScoringMechState {
        ACTIVE,
        WAITING_ON_SERVOS
    }

    public static class ElevatorParams {
        public double MIN_POS = 25;
        public double MAX_POS = 2000;
        public double SERVO_THRESHOLD = 800;
        public double SERVO_WAIT_THRESHOLD = 800;
        public double SERVO_WAIT_TIME = 0.5;
        public double TRANSIT_THRESHOLD = 50;
    }

    public static class IntakeParams {
        public double MOTOR_POWER = 0.9;
        public double RIGHT_UP_POS = 0.13;
        public double LEFT_UP_POS = 0.24;
        public double REVERSE_OFFSET = 0.20;
        public double DOWN_OFFSET = 0.48;
    }

    public static class LiftServoParams {
        public double TRANSIT_OFFSET = 0.05;
        public double INTAKE_RIGHT_POSITION = 0.75;
        public double INTAKE_LEFT_POSITION = 0.11;
        public double OUTTAKE_RIGHT_POSITION = 0.35;
        public double OUTTAKE_LEFT_POSITION = 0.51;
        public double HANG_RIGHT_POSITION = 0.25;
        public double HANG_LEFT_POSITION = 0.61;
    }

    public static class BucketParams {
        public double INTAKE_POSITION = 0.34;
        public double OUTTAKE_POSITION = 0.61;
        public double HANG_POSITION = 0.9;
        public double WHEEL_POWER = 1;
        public double BACK_COLOR_THRESHOLD = 0.5;
        public double FRONT_COLOR_THRESHOLD = 1.0;
    }
}