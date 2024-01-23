package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ServoPair;

@TeleOp
@Config
public class ServoPairTestOpMode extends LinearOpMode {
    public static double POSITION = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoPair servoPair = new ServoPair(
                hardwareMap.get(Servo.class, "rightIntake"),
                hardwareMap.get(Servo.class, "leftIntake")
        );

        waitForStart();

        while (opModeIsActive()) {
            servoPair.setPosition(POSITION);
        }
    }
}
