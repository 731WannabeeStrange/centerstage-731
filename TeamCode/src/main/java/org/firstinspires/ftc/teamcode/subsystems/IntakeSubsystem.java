package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intakeMotor;
    private final Servo rightServo;
    private final Servo leftServo;

    public static double intakeMotorPower = 0.75;
    public static double intakeServoRightUpPosition = 0.33;
    public static double intakeServoLeftUpPosition = 0.35;
    public static double intakeServoDownOffset = 0.4;

    private final TelemetryHandler telemetryHandler = TelemetryHandler.getInstance();

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // might change names in config
        rightServo = hardwareMap.get(Servo.class, "rightIntake");
        leftServo = hardwareMap.get(Servo.class, "leftIntake");

        leftServo.setDirection(Servo.Direction.REVERSE);

        // check this fits in 18in cube (probably won't, should have separate init pos)
        rightServo.setPosition(intakeServoRightUpPosition);
        leftServo.setPosition(intakeServoLeftUpPosition);

        register();
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = telemetryHandler.getCurrentPacket();
        packet.put("right intake servo position", rightServo.getPosition());
        packet.put("left intake servo position", leftServo.getPosition());
        packet.put("intake motor power", intakeMotor.getPower());
        telemetryHandler.updateCurrentPacket(packet);
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
