package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
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
    public static AutoParams AUTO_PARAMS = new AutoParams();

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
                if (Math.abs(getElevatorHeight() - ELEVATOR_PARAMS.TRANSIT_POS) < ELEVATOR_PARAMS.TRANSIT_ERROR_TOL) {
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

        telemetryHandler.addData("right outtake position", rightMotor.getCurrentPosition());
        telemetryHandler.addData("left outtake position", leftMotor.getCurrentPosition());

        //telemetryHandler.addData("right intake servo position", rightServo.getPosition());
        //telemetryHandler.addData("left intake servo position", leftServo.getPosition());
        //telemetryHandler.addData("intake motor power", intakeMotor.getPower());
    }

    private boolean areLiftMotorsBusy() {
        return !rightLiftController.atSetPoint() || !leftLiftController.atSetPoint();
    }

    public void reset() {
        setWheelState(WheelState.STOPPED);
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

    public void setIntakePosition(double position) {
        intakeServoPair.setPosition(position);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public boolean isFrontColorBlocked() {
        return frontColorSensor.getDistance(DistanceUnit.INCH) < BUCKET_PARAMS.FRONT_COLOR_THRESHOLD;
    }

    public boolean isBackColorBlocked() {
        return backColorSensor.getDistance(DistanceUnit.INCH) < BUCKET_PARAMS.BACK_COLOR_THRESHOLD;
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

    public Command resetIntake() {
        return new InstantCommand(() -> {
            setIntakePosition(INTAKE_PARAMS.UP_POS);
            setIntakePower(0);
            setWheelState(WheelState.STOPPED);
        });
    }

    public Command resetElevator() {
        return new CommandBase() {
            {
                //addRequirements(ScoringMech.this);
            }

            @Override
            public void initialize() {
                reset();
            }

            @Override
            public boolean isFinished() {
                return !isElevatorBusy() && getLiftServoState() == LiftServoState.INTAKE;
            }
        };
    }

    public Command raiseElevator(double height) {
        return new CommandBase() {
            {
                //addRequirements(ScoringMech.this);
            }

            @Override
            public void initialize() {
                setElevatorHeight(height);
            }

            @Override
            public void execute() {
                if (canLiftServosExtend()) {
                    setLiftServoState(LiftServoState.OUTTAKE);
                }
            }

            @Override
            public boolean isFinished() {
                return !isElevatorBusy();
            }
        };
    }

    public Command releasePixels(double releaseTime) {
        return new CommandBase() {
            {
                //addRequirements(ScoringMech.this);
            }
            private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            @Override
            public void initialize() {
                setWheelState(WheelState.OUTTAKE);
                elapsedTime.reset();
            }

            @Override
            public boolean isFinished() {
                return elapsedTime.time() > releaseTime;
            }

            @Override
            public void end(boolean interrupted) {
                setWheelState(WheelState.STOPPED);
            }
        };
    }

    public Command scoreGround() {
        return new CommandBase() {
            {
                //addRequirements(ScoringMech.this);
            }
            private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            @Override
            public void initialize() {
                setLiftServoState(LiftServoState.SCORE_GROUND);
                setIntakePosition(INTAKE_PARAMS.REVERSE_POS);
                setIntakePower(-INTAKE_PARAMS.OUTTAKE_POWER);
                elapsedTime.reset();
            }

            @Override
            public boolean isFinished() {
                return elapsedTime.time() > AUTO_PARAMS.SCORE_GROUND_TIME;
            }

            @Override
            public void end(boolean interrupted) {
                setIntakePosition(INTAKE_PARAMS.UP_POS);
                setIntakePower(0);
                setWheelState(WheelState.STOPPED);
            }
        };
    }

    public Command dropIntake() {
        return new InstantCommand(() -> setIntakePosition(AUTO_PARAMS.INTAKE_DROP_POS));
    }

    public Command spinIntake() {
        return new CommandBase() {
            {
                //addRequirements(ScoringMech.this);
            }

            @Override
            public void initialize() {
                setIntakePower(INTAKE_PARAMS.MOTOR_POWER);
                setWheelState(WheelState.INTAKE);
            }

            @Override
            public boolean isFinished() {
                return isFrontColorBlocked() && isBackColorBlocked();
            }

            @Override
            public void end(boolean interrupted) {
                setIntakePosition(INTAKE_PARAMS.UP_POS);
                setIntakePower(0);
                setWheelState(WheelState.STOPPED);
            }
        };
    }

    public Command spinIntake(double timeout) {
        return new CommandBase() {
            {
                //addRequirements(ScoringMech.this);
            }
            private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            @Override
            public void initialize() {
                setIntakePower(INTAKE_PARAMS.MOTOR_POWER);
                setWheelState(WheelState.INTAKE);
                elapsedTime.reset();
            }

            @Override
            public boolean isFinished() {
                return (isFrontColorBlocked() && isBackColorBlocked()) || elapsedTime.time() > timeout;
            }

            @Override
            public void end(boolean interrupted) {
                setIntakePosition(INTAKE_PARAMS.UP_POS);
                setIntakePower(0);
                setWheelState(WheelState.STOPPED);
            }
        };
    }

    public Command flushIntake() {
        return new CommandBase() {
            {
                //addRequirements(ScoringMech.this);
            }
            private FlushState flushState = FlushState.ARM_MOVING;
            private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            @Override
            public void initialize() {
                setIntakePosition(INTAKE_PARAMS.REVERSE_POS);
                elapsedTime.reset();
            }

            @Override
            public void execute() {
                switch (flushState) {
                    case ARM_MOVING:
                        if (elapsedTime.time() > AUTO_PARAMS.FLUSH_ARM_TIME) {
                            setIntakePower(-INTAKE_PARAMS.OUTTAKE_POWER);
                            flushState = FlushState.FLUSHING;
                        }
                        break;
                    case FLUSHING:
                        if (elapsedTime.time() > AUTO_PARAMS.FLUSH_WHEEL_TIME) {
                            setIntakePower(0);
                            flushState = FlushState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;
                }
            }

            @Override
            public boolean isFinished() {
                return flushState == FlushState.IDLE;
            }

            @Override
            public void end(boolean interrupted) {
                setIntakePosition(INTAKE_PARAMS.UP_POS);
                //setWheelState(WheelState.STOPPED);
                flushState = FlushState.ARM_MOVING;
            }
        };
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

    private enum FlushState {
        ARM_MOVING,
        FLUSHING,
        IDLE
    }

    public static class ElevatorParams {
        public double kP = 0.006;
        public double kI = 0;
        public double kD = 0;
        public double kF = 2e-7;
        public double POSITION_TOLERANCE = 15;
        public double TRANSIT_ERROR_TOL = 30;
        public double MIN_POS = 25;
        public double MAX_POS = 3000;
        public double SERVO_THRESHOLD = 800;
        public double SERVO_WAIT_TIME = 0.1;
        public double TRANSIT_POS = 460;
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

    public static class AutoParams {
        public double SCORE_GROUND_TIME = 0.25;
        public double FLUSH_ARM_TIME = 0.25;
        public double FLUSH_WHEEL_TIME = 1;
        public double INTAKE_DROP_POS = 0.73;
    }
}