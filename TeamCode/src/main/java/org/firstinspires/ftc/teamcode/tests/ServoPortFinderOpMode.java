package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoPortFinderOpMode extends LinearOpMode {
    public static double POSITION = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo mysteryServo = hardwareMap.get(Servo.class, "bucket");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            mysteryServo.setPosition(POSITION);
        }
    }
}
