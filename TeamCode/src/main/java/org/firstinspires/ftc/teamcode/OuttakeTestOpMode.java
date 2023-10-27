package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Outtake Test", group = "test")
@Config
public class OuttakeTestOpMode extends LinearOpMode {

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
        outtake = hardwareMap.get(Servo.class, "outtake");

        outtake.setPosition(0);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
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
            telemetry.update();
        }
    }
}
