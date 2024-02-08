package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@TeleOp
@Config
public class LiftPositionerOpMode extends LinearOpMode {
    public static double POSITION = 0;
    public static double liftP = 0.007;
    public static double liftI = 0;
    public static double liftD = 0;
    public static double liftF = 0.0000002;
    public static double POSITION_TOLERANCE = 15;

    private final PIDFController rightLiftController = new PIDFController(liftP, liftI, liftD, liftF);
    private final PIDFController leftLiftController = new PIDFController(liftP, liftI, liftD, liftF);
    private DcMotorEx leftMotor, rightMotor;

    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightOuttake");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            rightLiftController.setTolerance(POSITION_TOLERANCE);
            leftLiftController.setTolerance(POSITION_TOLERANCE);
            rightLiftController.setPIDF(liftP, liftI, liftD, liftF);
            leftLiftController.setPIDF(liftP, liftI, liftD, liftF);

            rightMotor.setPower(rightLiftController.calculate(rightMotor.getCurrentPosition(), POSITION));
            leftMotor.setPower(leftLiftController.calculate(leftMotor.getCurrentPosition(), POSITION));

            telemetryHandler.addData("right motor pos", rightMotor.getCurrentPosition());
            telemetryHandler.addData("left motor pos", leftMotor.getCurrentPosition());
            telemetryHandler.addData("right controller at setpoint", rightLiftController.atSetPoint());
            telemetryHandler.addData("left controller at setpoint", leftLiftController.atSetPoint());
            telemetryHandler.update();
        }
    }
}
