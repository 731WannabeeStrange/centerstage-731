package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class LiftPositionerOpMode extends LinearOpMode {
    public static double INTAKE_LIFT_POSITION = 0.77;
    public static double INTAKE_BUCKET_POSITION = 0.67;
    public static double OUTTAKE_LIFT_POSITION = 0.35;
    public static double OUTTAKE_BUCKET_POSITION = 0.88;

    private enum LiftState {
        INTAKE,
        OUTTAKE
    }
    private LiftState liftState = LiftState.INTAKE;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo rightLiftServo = hardwareMap.get(Servo.class, "rightLift");
        Servo bucketServo = hardwareMap.get(Servo.class, "bucket");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                liftState = LiftState.INTAKE;
            } else if (gamepad1.y) {
                liftState = LiftState.OUTTAKE;
            }

            switch (liftState) {
                case INTAKE:
                    rightLiftServo.setPosition(INTAKE_LIFT_POSITION);
                    bucketServo.setPosition(INTAKE_BUCKET_POSITION);
                    break;
                case OUTTAKE:
                    rightLiftServo.setPosition(OUTTAKE_LIFT_POSITION);
                    bucketServo.setPosition(OUTTAKE_BUCKET_POSITION);
                    break;
            }
        }
    }
}
