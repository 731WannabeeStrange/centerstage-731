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
import org.firstinspires.ftc.teamcode.utils.RangeController;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.caching.CachingServo;

@Config
public class ScoringMech extends SubsystemBase {
    public static TrapezoidProfile.Constraints liftConstraints = new TrapezoidProfile.Constraints(2500, 2500);
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0.004, 0, 0);
    private final ProfiledPIDController liftController = new ProfiledPIDController(
            liftCoefficients.p,
            liftCoefficients.i,
            liftCoefficients.d,
            liftConstraints
    );
    private final DcMotorEx leftMotor, rightMotor, intakeMotor;
    private final Servo rightServo, leftServo, bucketServo, leftLiftServo, rightLiftServo;
    private final CRServo wheelServo;
    private final RevColorSensorV3 frontColorSensor, backColorSensor;
    private final TelemetryHandler telemetryHandler;

    public static double MIN_POS = 0;
    public static double MAX_POS = 2000;
    public static double SERVO_THRESHOLD = 800;

    private double currentTarget = MIN_POS;

    public static double INTAKE_RIGHT_LIFT_POSITION = 0.75;
    public static double INTAKE_LEFT_LIFT_POSITION = 0.11;
    public static double INTAKE_BUCKET_POSITION = 0.34;
    public static double OUTTAKE_RIGHT_LIFT_POSITION = 0.35;
    public static double OUTTAKE_LEFT_LIFT_POSITION = 0.51;
    public static double OUTTAKE_BUCKET_POSITION = 0.61;
    public static double LIFT_RIGHT_LIFT_POSITION = 0.25;
    public static double LIFT_LEFT_LIFT_POSITION = 0.61;
    public static double LIFT_BUCKET_POSITION = 0.9;
    public static double WHEEL_POWER = 1;

    public static double INTAKE_MOTOR_POWER = 0.9;
    public static double INTAKE_RIGHT_SERVO_UP_POS = 0.13;
    public static double INTAKE_LEFT_SERVO_UP_POS = 0.24;
    public static double INTAKE_SERVO_DOWN_OFFSET = 0.48;

    public enum WheelState {
        INTAKE,
        OUTTAKE,
        STOPPED
    }

    public enum LiftServoState {
        INTAKE,
        OUTTAKE,
        LIFT
    }
    private LiftServoState liftServoState = LiftServoState.INTAKE;

    public ScoringMech(HardwareMap hardwareMap, TelemetryHandler telemetryHandler) {
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

        intakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        rightServo = new CachingServo(hardwareMap.get(Servo.class, "rightIntake"));
        leftServo = new CachingServo(hardwareMap.get(Servo.class, "leftIntake"));

        leftServo.setDirection(Servo.Direction.REVERSE);

        // check this fits in 18in cube (probably won't, should have separate init pos)
        rightServo.setPosition(INTAKE_RIGHT_SERVO_UP_POS);
        leftServo.setPosition(INTAKE_LEFT_SERVO_UP_POS);

        this.telemetryHandler = telemetryHandler;
    }

    @Override
    public void periodic() {
        rightMotor.setPower(liftController.calculate(rightMotor.getCurrentPosition(), currentTarget));
        leftMotor.setPower(liftController.calculate(leftMotor.getCurrentPosition(), currentTarget));

        telemetryHandler.addData("front color distance (in)", frontColorSensor.getDistance(DistanceUnit.INCH));
        telemetryHandler.addData("back color distance (in)", backColorSensor.getDistance(DistanceUnit.INCH));
        telemetryHandler.addData("num pixels in bucket", getNumPixelsInBucket());
        telemetryHandler.addData("right outtake position", rightMotor.getCurrentPosition());
        telemetryHandler.addData("left outtake position", leftMotor.getCurrentPosition());

        telemetryHandler.addData("right intake servo position", rightServo.getPosition());
        telemetryHandler.addData("left intake servo position", leftServo.getPosition());
        telemetryHandler.addData("intake motor power", intakeMotor.getPower());
    }

    public void setElevatorHeight(double fraction) {
        currentTarget = RangeController.getPositionFromRange(fraction, MIN_POS, MAX_POS);
    }

    public void setLiftServoState(LiftServoState liftServoState) {
        if (liftServoState != this.liftServoState) {
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
                case LIFT:
                    rightLiftServo.setPosition(LIFT_RIGHT_LIFT_POSITION);
                    leftLiftServo.setPosition(LIFT_LEFT_LIFT_POSITION);
                    bucketServo.setPosition(LIFT_BUCKET_POSITION);
                    break;
            }
            this.liftServoState = liftServoState;
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

    public void startIntake() {
        intakeMotor.setPower(INTAKE_MOTOR_POWER);
        rightServo.setPosition(INTAKE_RIGHT_SERVO_UP_POS + INTAKE_SERVO_DOWN_OFFSET);
        leftServo.setPosition(INTAKE_LEFT_SERVO_UP_POS + INTAKE_SERVO_DOWN_OFFSET);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        rightServo.setPosition(INTAKE_RIGHT_SERVO_UP_POS);
        leftServo.setPosition(INTAKE_LEFT_SERVO_UP_POS);
    }

    public double getFrontColor() {
        return frontColorSensor.getDistance(DistanceUnit.INCH);
    }

    public double getBackColor() {
        return backColorSensor.getDistance(DistanceUnit.INCH);
    }

    public int getNumPixelsInBucket() {
        double frontColor = getFrontColor();
        double backColor = getBackColor();
        if (frontColor < 1.0 && backColor < 0.5) {
            return 2;
        } else if (frontColor < 1.0) {
            return 1;
        } else {
            return 0;
        }
    }

    public LiftServoState getLiftServoState() {
        return liftServoState;
    }

    public boolean canLiftServosExtend() {
        return Math.abs(rightMotor.getCurrentPosition()) > SERVO_THRESHOLD;
    }

    public boolean isElevatorBusy() {
        return !liftController.atSetpoint();
    }
}