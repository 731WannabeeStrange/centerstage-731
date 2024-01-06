package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.caching.CachingServo;

@Config
public class Intake extends SubsystemBase {
    public static double intakeMotorPower = 0.7;
    public static double intakeServoRightUpPosition = 0.38;
    public static double intakeServoLeftUpPosition = 0.35;
    public static double intakeServoDownOffset = 0.43;
    private final DcMotorEx intakeMotor;
    private final Servo rightServo;
    private final Servo leftServo;
    private final TelemetryHandler telemetryHandler;

    public Intake(HardwareMap hardwareMap, TelemetryHandler telemetryHandler) {
        intakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // might change names in config
        rightServo = new CachingServo(hardwareMap.get(Servo.class, "rightIntake"));
        leftServo = new CachingServo(hardwareMap.get(Servo.class, "leftIntake"));

        leftServo.setDirection(Servo.Direction.REVERSE);

        // check this fits in 18in cube (probably won't, should have separate init pos)
        rightServo.setPosition(intakeServoRightUpPosition);
        leftServo.setPosition(intakeServoLeftUpPosition);

        this.telemetryHandler = telemetryHandler;
        register();
    }

    @Override
    public void periodic() {
        telemetryHandler.addData("right intake servo position", rightServo.getPosition());
        telemetryHandler.addData("left intake servo position", leftServo.getPosition());
        telemetryHandler.addData("intake motor power", intakeMotor.getPower());
    }

    public void start() {
        intakeMotor.setPower(intakeMotorPower);
        rightServo.setPosition(intakeServoRightUpPosition + intakeServoDownOffset);
        leftServo.setPosition(intakeServoLeftUpPosition + intakeServoDownOffset);
    }

    public void stop() {
        intakeMotor.setPower(0);
        rightServo.setPosition(intakeServoRightUpPosition);
        leftServo.setPosition(intakeServoLeftUpPosition);
    }
}
