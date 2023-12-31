package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(group = "test")
@Config
public class MecanumTestOpMode extends LinearOpMode {
    public static double P = 0.04;
    public static double I = 0;
    public static double D = 0;
    private final ElapsedTime eTime = new ElapsedTime();
    public double leftStickY;
    public double leftStickX;
    public double rightStickX;
    public double FL_power;
    public double FR_power;
    public double RL_power;
    public double RR_power;
    public double newForward;
    public double newStrafe;
    public double denominator;
    public Orientation angles;
    private double integral, previous_error = 0;
    private BNO055IMU imu;
    private DcMotor fr;
    private DcMotor rr;
    private DcMotor fl;
    private DcMotor rl;
    private double error;
    private double errorMin;
    private double desiredAngle = 0;
    private String turnState = "auto";
    private driveMode driveState = driveMode.AUTO_CONTROL;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        parameters.mode = BNO055IMU.SensorMode.IMU;
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        fr = hardwareMap.get(DcMotor.class, "frMotor");
        rr = hardwareMap.get(DcMotor.class, "rrMotor");
        fl = hardwareMap.get(DcMotor.class, "flMotor");
        rl = hardwareMap.get(DcMotor.class, "rlMotor");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        eTime.reset();

        while (opModeIsActive()) {
            controls();
        }
    }

    public void controls() {
        holonomicFormula();
        telemetry.update();

        eTime.reset();
    }

    public void getJoyValues() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.right_stick_x * 1.1;
        rightStickX = gamepad1.left_stick_x;

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi / 180;
        newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);
    }

    public void holonomicFormula() {
        double time = eTime.time();
        getJoyValues();

        if (desiredAngle - angles.firstAngle < 0) {
            errorMin = Math.min(Math.abs(desiredAngle - angles.firstAngle), Math.abs(desiredAngle - angles.firstAngle + 360));
        } else {
            errorMin = Math.min(Math.abs(desiredAngle - angles.firstAngle), Math.abs(desiredAngle - angles.firstAngle - 360));
        }

        if (errorMin == Math.abs(desiredAngle - angles.firstAngle)) {
            error = desiredAngle - angles.firstAngle;
        } else if (errorMin == Math.abs(desiredAngle - angles.firstAngle - 360)) {
            error = desiredAngle - angles.firstAngle - 360;
        } else if (errorMin == Math.abs(desiredAngle - angles.firstAngle + 360)) {
            error = desiredAngle - angles.firstAngle + 360;
        }

        telemetry.addData("Turn Error", error);

        integral += (error * time);
        eTime.reset();

        double derivative = (error - previous_error) / time;
        double rcw = P * -error + I * integral + D * derivative;

        previous_error = error;

        telemetry.addData("Read angle", angles.firstAngle);
        telemetry.addData("RCW", rcw);
        telemetry.addData("Desired Angle", desiredAngle);
        telemetry.addData("rightStickX", rightStickX);

        switch (driveState) {
            case AUTO_CONTROL:
                if (!gamepad1.right_bumper) {
                    if (gamepad1.dpad_up) {
                        desiredAngle = 0;
                    }
                    if (gamepad1.dpad_right) {
                        desiredAngle = 270;
                    }
                    if (gamepad1.dpad_down) {
                        desiredAngle = 180;
                    }
                    if (gamepad1.dpad_left) {
                        desiredAngle = 90;
                    }
                }
                if (rightStickX != 0) {
                    driveState = driveMode.DRIVER_CONTROLLED;
                }
                turnState = "auto";
                //denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rcw), 1);
                FL_power = (-newForward + newStrafe + rcw);// / denominator;
                RL_power = (-newForward - newStrafe + rcw);// / denominator;
                FR_power = (-newForward - newStrafe - rcw);// / denominator;
                RR_power = (-newForward + newStrafe - rcw);// / denominator;
                break;
            case DRIVER_CONTROLLED:
                turnState = "driver";
                denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rightStickX), 1);
                FL_power = (-newForward + newStrafe + rightStickX) / denominator;
                RL_power = (-newForward - newStrafe + rightStickX) / denominator;
                FR_power = (-newForward - newStrafe - rightStickX) / denominator;
                RR_power = (-newForward + newStrafe - rightStickX) / denominator;
                if (!gamepad1.right_bumper) {
                    if (gamepad1.dpad_up) {
                        desiredAngle = 0;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                    if (gamepad1.dpad_right) {
                        desiredAngle = 270;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                    if (gamepad1.dpad_down) {
                        desiredAngle = 180;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                    if (gamepad1.dpad_left) {
                        desiredAngle = 90;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                }
                break;
        }

        telemetry.addData("turnState", turnState);

        if (gamepad1.left_bumper) {
            FL_power /= 4;
            FR_power /= 4;
            RL_power /= 4;
            RR_power /= 4;
        }

        telemetry.addData("FLpower", FL_power);
        telemetry.addData("FRpower", FR_power);
        telemetry.addData("RLpower", RL_power);
        telemetry.addData("RRpower", RR_power);

        fl.setPower(FL_power);
        fr.setPower(FR_power);
        rl.setPower(RL_power);
        rr.setPower(RR_power);
    }

    private enum driveMode {
        DRIVER_CONTROLLED,
        AUTO_CONTROL
    }
}