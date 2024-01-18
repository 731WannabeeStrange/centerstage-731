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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Config
public class Elevator extends SubsystemBase {
    public static TrapezoidProfile.Constraints liftConstraints = new TrapezoidProfile.Constraints(2500, 2500);
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0.004, 0, 0);
    private final ProfiledPIDController liftController = new ProfiledPIDController(
            liftCoefficients.p,
            liftCoefficients.i,
            liftCoefficients.d,
            liftConstraints
    );
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private final Servo bucketServo;
    private final Servo leftLiftServo;
    private final Servo rightLiftServo;
    private final CRServo wheelServo;
    private final RevColorSensorV3 frontColorSensor;
    private final RevColorSensorV3 backColorSensor;
    private final TelemetryHandler telemetryHandler;

    public static double IDLE_POS = 0;
    public static double LIFT_DOWN_POS = 200;
    public static double MIN_SCORING_POS = 800;
    public static double LIFT_UP_POS = 1600;
    public static double MAX_SCORING_POS = 2000;

    private double currentTarget = IDLE_POS;

    public static double INTAKE_RIGHT_LIFT_POSITION = 0.75;
    public static double INTAKE_LEFT_LIFT_POSITION = 0.11;
    public static double INTAKE_BUCKET_POSITION = 0.34;
    public static double OUTTAKE_RIGHT_LIFT_POSITION = 0.35;
    public static double OUTTAKE_LEFT_LIFT_POSITION = 0.51;
    public static double OUTTAKE_BUCKET_POSITION = 0.61;
    public static double LIFT_RIGHT_LIFT_POSITION = 0.55;
    public static double LIFT_LEFT_LIFT_POSITION = 0.31;
    public static double LIFT_BUCKET_POSITION = 0.61;
    public static double WHEEL_POWER = 1;

    public enum WheelState {
        INTAKE,
        OUTTAKE,
        STOPPED
    }

    public enum LiftServoState {
        INTAKE,
        OUTTAKE
    }

    public enum ElevatorState {
        IDLE,
        LIFT_DOWN,
        MINIMUM,
        LIFT_UP,
        MAXIMUM
    }

    public ElevatorState elevatorState = ElevatorState.IDLE;

    public Elevator(HardwareMap hardwareMap, TelemetryHandler telemetryHandler) {
        liftController.setTolerance(20);

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightOuttake");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketServo = hardwareMap.get(Servo.class, "bucket");
        leftLiftServo = hardwareMap.get(Servo.class, "leftLift");
        rightLiftServo = hardwareMap.get(Servo.class, "rightLift");
        wheelServo = hardwareMap.get(CRServo.class, "wheel");

        rightLiftServo.setPosition(INTAKE_RIGHT_LIFT_POSITION);
        leftLiftServo.setPosition(INTAKE_LEFT_LIFT_POSITION);
        bucketServo.setPosition(INTAKE_BUCKET_POSITION);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        backColorSensor = hardwareMap.get(RevColorSensorV3.class, "backColor");

        this.telemetryHandler = telemetryHandler;
    }

    @Override
    public void periodic() {
        rightMotor.setPower(liftController.calculate(rightMotor.getCurrentPosition(), currentTarget));
        leftMotor.setPower(liftController.calculate(leftMotor.getCurrentPosition(), currentTarget));

        telemetryHandler.addData("front color distance (in)", frontColorSensor.getDistance(DistanceUnit.INCH));
        telemetryHandler.addData("back color distance (in)", backColorSensor.getDistance(DistanceUnit.INCH));
        telemetryHandler.addData("right outtake position", rightMotor.getCurrentPosition());
        telemetryHandler.addData("left outtake position", leftMotor.getCurrentPosition());
    }

    public void setElevatorHeight(ElevatorState elevatorState) {
        switch (elevatorState) {
            case IDLE:
                currentTarget = IDLE_POS;
                setLiftServoState(LiftServoState.INTAKE);
                break;
            case LIFT_DOWN:
                currentTarget = LIFT_DOWN_POS;
                setLiftServoState(LiftServoState.INTAKE);
                break;
            case MINIMUM:
                currentTarget = MIN_SCORING_POS;
                break;
            case LIFT_UP:
                currentTarget = LIFT_UP_POS;
                break;
            case MAXIMUM:
                currentTarget = MAX_SCORING_POS;
                break;
        }
        this.elevatorState = elevatorState;
    }

    public void setLiftServoState(LiftServoState liftServoState) {
        switch (liftServoState) {
            case INTAKE:
                rightLiftServo.setPosition(INTAKE_RIGHT_LIFT_POSITION);
                leftLiftServo.setPosition(INTAKE_LEFT_LIFT_POSITION);
                bucketServo.setPosition(INTAKE_BUCKET_POSITION);
                break;
            case OUTTAKE:
                rightLiftServo.setPosition(OUTTAKE_RIGHT_LIFT_POSITION);
                leftLiftServo.setPosition(OUTTAKE_LEFT_LIFT_POSITION);
                bucketServo.setPosition(OUTTAKE_BUCKET_POSITION);
                break;
        }
    }

    public void setWheelState(WheelState wheelState) {
        switch (wheelState) {
            case INTAKE:
                wheelServo.setPower(-WHEEL_POWER);
                break;
            case OUTTAKE:
                wheelServo.setPower(WHEEL_POWER);
                break;
            case STOPPED:
                wheelServo.setPower(0);
                break;
        }
    }

    public double getFrontColor() {
        return frontColorSensor.getDistance(DistanceUnit.INCH);
    }

    public double getBackColor() {
        return backColorSensor.getDistance(DistanceUnit.INCH);
    }

    public double getElevatorHeight() {
        return rightMotor.getCurrentPosition();
    }

    public boolean isBucketFull() {
        return getFrontColor() < 1.0 && getBackColor() < 0.5;
    }

    public boolean isInIntakePosition() {
        return elevatorState == ElevatorState.IDLE && liftController.atSetpoint();
    }

    public boolean isInScoringPosition() {
        return (elevatorState == ElevatorState.MINIMUM || elevatorState == ElevatorState.MAXIMUM) && liftController.atSetpoint();
    }

    public boolean isLiftUp() {
        return elevatorState == ElevatorState.LIFT_UP && liftController.atSetpoint();
    }

    public boolean isElevatorBusy() {
        return !liftController.atSetpoint();
    }

    public void sendLiftUp() {
        setElevatorHeight(ElevatorState.LIFT_UP);
        rightLiftServo.setPosition(LIFT_RIGHT_LIFT_POSITION);
        leftLiftServo.setPosition(LIFT_LEFT_LIFT_POSITION);
        bucketServo.setPosition(LIFT_BUCKET_POSITION);
    }

    public void sendLiftDown() {
        setElevatorHeight(ElevatorState.IDLE);
    }
}