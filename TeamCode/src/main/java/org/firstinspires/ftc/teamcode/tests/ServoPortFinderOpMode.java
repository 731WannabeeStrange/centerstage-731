package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Config
public class ServoPortFinderOpMode extends LinearOpMode {
    public static double POWER = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo mysteryServo = hardwareMap.get(CRServo.class, "wheel");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            mysteryServo.setPower(POWER);
        }
    }
}
