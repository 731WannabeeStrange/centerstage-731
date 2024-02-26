package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class DronePositionerOpMode extends LinearOpMode {
    public static double POSITION = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo drone = hardwareMap.get(Servo.class, "droneServo");

        waitForStart();

        while (opModeIsActive()) {
            drone.setPosition(POSITION);
        }
    }
}
