package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ServoPair;

@TeleOp
@Config
public class LiftPositionerOpMode extends LinearOpMode {
    public static double INTAKE_LIFT_POS = 0.66;
    public static double INTAKE_BUCKET_POS = 0.32;
    public static double OUTTAKE_LIFT_POS = 0.2;
    public static double OUTTAKE_BUCKET_POS = 0.57;
    public static double TRANSIT_LIFT_POS = 0.5;
    public static double TRANSIT_BUCKET_POS = 0.55;
    public static double HANG_LIFT_POS = 0.2;
    public static double HANG_BUCKET_POS = 0.83;

    private enum LiftState {
        INTAKE,
        TRANSIT,
        OUTTAKE,
        HANG
    }

    private LiftState liftState = LiftState.INTAKE;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoPair liftServoPair = new ServoPair(
                hardwareMap.get(Servo.class, "rightLift"),
                hardwareMap.get(Servo.class, "leftLift")
        );
        Servo bucketServo = hardwareMap.get(Servo.class, "bucket");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                liftState = LiftState.INTAKE;
            } else if (gamepad1.b) {
                liftState = LiftState.TRANSIT;
            } else if (gamepad1.y) {
                liftState = LiftState.OUTTAKE;
            } else if (gamepad1.x) {
                liftState = LiftState.HANG;
            }

            switch (liftState) {
                case INTAKE:
                    liftServoPair.setPosition(INTAKE_LIFT_POS);
                    bucketServo.setPosition(INTAKE_BUCKET_POS);
                    break;
                case TRANSIT:
                    liftServoPair.setPosition(TRANSIT_LIFT_POS);
                    bucketServo.setPosition(TRANSIT_BUCKET_POS);
                    break;
                case OUTTAKE:
                    liftServoPair.setPosition(OUTTAKE_LIFT_POS);
                    bucketServo.setPosition(OUTTAKE_BUCKET_POS);
                    break;
                case HANG:
                    liftServoPair.setPosition(HANG_LIFT_POS);
                    bucketServo.setPosition(HANG_BUCKET_POS);
                    break;
            }

            telemetry.addData("lift state", liftState);
            telemetry.update();
        }
    }
}
