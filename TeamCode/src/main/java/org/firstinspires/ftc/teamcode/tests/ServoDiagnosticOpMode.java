package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoDiagnosticOpMode extends OpMode {
    private Servo intakeServoRight, intakeServoLeft;
    private final MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    public static double servoRightUpPos = 0.33;
    public static double servoLeftUpPos = 0.35;
    public static double offset = 0.4;

    @Override
    public void init() {
        intakeServoRight = hardwareMap.get(Servo.class, "rightIntake");
        intakeServoLeft = hardwareMap.get(Servo.class, "leftIntake");
        intakeServoLeft.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        intakeServoRight.setPosition(servoRightUpPos + offset);
        intakeServoLeft.setPosition(servoLeftUpPos + offset);

        multipleTelemetry.addData("intakeServoRight position", intakeServoRight.getPosition());
        multipleTelemetry.addData("intakeServoLeft position", intakeServoLeft.getPosition());
        multipleTelemetry.update();
    }
}
