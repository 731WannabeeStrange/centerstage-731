package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Config
public class Elevator extends SubsystemBase {
    public static TrapezoidProfile.Constraints liftConstraints = new TrapezoidProfile.Constraints(2500, 2500);
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0.005, 0, 0);
    private final ProfiledPIDController liftController = new ProfiledPIDController(
            liftCoefficients.p,
            liftCoefficients.i,
            liftCoefficients.d,
            liftConstraints
    );
    private DcMotorEx leftMotor, rightMotor;

    private final Servo bucketServo;
    private final Servo leftLiftServo;
    private final Servo rightLiftServo;
    private final Servo wheelServo;
    private final RevColorSensorV3 frontColorSensor;
    private final RevColorSensorV3 backColorSensor;
    private final TelemetryHandler telemetryHandler;

    public static double IDLE_POS = 500;
    public static double MIN_SCORING_POS = 1000;
    public static double MAX_SCORING_POS = 2000;

    private double currentTarget = IDLE_POS;

    public static double INTAKE_LIFT_POSITION = 0.77;
    public static double INTAKE_BUCKET_POSITION = 0.67;
    public static double OUTTAKE_LIFT_POSITION = 0.35;
    public static double OUTTAKE_BUCKET_POSITION = 0.88;
    public enum LiftState {
        INTAKE,
        OUTTAKE
    }

    public Elevator(HardwareMap hardwareMap, TelemetryHandler telemetryHandler) {
        //leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        //rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");

        //leftMotor.setDirection(DcMotor.Direction.REVERSE);

        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketServo = hardwareMap.get(Servo.class, "bucket");
        leftLiftServo = hardwareMap.get(Servo.class, "leftLift");
        rightLiftServo = hardwareMap.get(Servo.class, "rightLift");
        wheelServo = hardwareMap.get(Servo.class, "wheel");

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        backColorSensor = hardwareMap.get(RevColorSensorV3.class, "backColor");

        this.telemetryHandler = telemetryHandler;
        register();
    }

    @Override
    public void periodic() {
        //rightMotor.setPower(liftController.calculate(rightMotor.getCurrentPosition(), currentTarget));
        telemetryHandler.addData("front color distance (in)", frontColorSensor.getDistance(DistanceUnit.INCH));
        telemetryHandler.addData("back color distance (in)", backColorSensor.getDistance(DistanceUnit.INCH));
    }

    public void setLiftState(LiftState liftState) {
        switch (liftState) {
            case INTAKE:
                rightLiftServo.setPosition(INTAKE_LIFT_POSITION);
                bucketServo.setPosition(INTAKE_BUCKET_POSITION);
                break;
            case OUTTAKE:
                rightLiftServo.setPosition(OUTTAKE_LIFT_POSITION);
                bucketServo.setPosition(OUTTAKE_BUCKET_POSITION);
                break;
        }
    }

    public double getFrontColor() {
        return frontColorSensor.getDistance(DistanceUnit.INCH);
    }

    public double getBackColor() {
        return backColorSensor.getDistance(DistanceUnit.INCH);
    }

    public void goToMaxScoringPos() {
        currentTarget = MAX_SCORING_POS;
    }

    public void goToMinScoringPos() {
        currentTarget = MIN_SCORING_POS;
    }

    public void goToIdlePose() {
        currentTarget = IDLE_POS;
    }
}