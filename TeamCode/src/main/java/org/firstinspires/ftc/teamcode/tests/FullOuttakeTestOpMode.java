package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "test")
@Config
public class FullOuttakeTestOpMode extends LinearOpMode {

    public static double slidePower = 0.3;
    private DcMotorEx slide;

    private enum ServoPosition {
        DOWN,
        MID,
        UP
    }

    private ServoPosition position = ServoPosition.DOWN;
    private Servo outtake;
    public static double downPos = 0, midPos = 0.5, upPos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setPosition(0);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("outtake position", outtake.getPosition());
            if (gamepad1.dpad_down) {
                position = ServoPosition.DOWN;
            } else if (gamepad1.dpad_right) {
                position = ServoPosition.MID;
            } else if (gamepad1.dpad_up) {
                position = ServoPosition.UP;
            }

            switch (position) {
                case DOWN:
                    outtake.setPosition(downPos);
                    break;
                case MID:
                    outtake.setPosition(midPos);
                    break;
                case UP:
                    outtake.setPosition(upPos);
                    break;
            }

            if (gamepad1.b) {
                slide.setPower(slidePower);
            } else if (gamepad1.x) {
                slide.setPower(-slidePower);
            } else {
                slide.setPower(0);
            }

            telemetry.addData("slide position", slide.getCurrentPosition());

            telemetry.update();
        }
    }
}