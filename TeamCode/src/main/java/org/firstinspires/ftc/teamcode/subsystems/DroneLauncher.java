package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Config
public class DroneLauncher extends SubsystemBase {
    public static double LAUNCH_VELOCITY = 1300;
    public static double LAUNCH_POS = 0.3;
    public static double ARMED_POS = 0;
    public static double VELOCITY_TOLERANCE = 15;
    public static double LAUNCH_TIME = 0.5;

    private final Servo droneServo;
    private final DcMotorEx droneMotor;

    private final TelemetryHandler telemetryHandler;

    public DroneLauncher(HardwareMap hardwareMap, TelemetryHandler telemetryHandler) {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        droneMotor = hardwareMap.get(DcMotorEx.class, "droneMotor");

        this.telemetryHandler = telemetryHandler;

        droneServo.setPosition(ARMED_POS);
    }

    public Command launchDrone() {
        return new CommandBase() {
            {
                addRequirements(DroneLauncher.this);
            }
            private DroneLaunchState state = DroneLaunchState.SPINNING_UP;
            private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            @Override
            public void initialize() {
                droneMotor.setVelocity(LAUNCH_VELOCITY);
            }

            @Override
            public void execute() {
                switch (state) {
                    case SPINNING_UP:
                        if (Math.abs(droneMotor.getVelocity() - LAUNCH_VELOCITY) < VELOCITY_TOLERANCE) {
                            droneServo.setPosition(LAUNCH_POS);
                            state = DroneLaunchState.LAUNCHING;
                            elapsedTime.reset();
                        }
                        break;
                    case LAUNCHING:
                        if (elapsedTime.time() > LAUNCH_TIME) {
                            droneMotor.setVelocity(0);
                            droneServo.setPosition(ARMED_POS);
                            state = DroneLaunchState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;
                }

                telemetryHandler.addData("Drone run state", state);
                telemetryHandler.addData("Drone launcher velocity", droneMotor.getVelocity());
            }

            @Override
            public boolean isFinished() {
                return state == DroneLaunchState.IDLE;
            }

            @Override
            public void end(boolean interruptible) {
                state = DroneLaunchState.SPINNING_UP;
            }
        };
    }

    public Command launchDrone(double velocity) {
        return new CommandBase() {
            {
                addRequirements(DroneLauncher.this);
            }
            private DroneLaunchState state = DroneLaunchState.SPINNING_UP;
            private final ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            @Override
            public void initialize() {
                droneMotor.setVelocity(velocity);
            }

            @Override
            public void execute() {
                switch (state) {
                    case SPINNING_UP:
                        if (Math.abs(droneMotor.getVelocity() - velocity) < VELOCITY_TOLERANCE) {
                            droneServo.setPosition(LAUNCH_POS);
                            state = DroneLaunchState.LAUNCHING;
                            elapsedTime.reset();
                        }
                        break;
                    case LAUNCHING:
                        if (elapsedTime.time() > LAUNCH_TIME) {
                            droneMotor.setVelocity(0);
                            droneServo.setPosition(ARMED_POS);
                            state = DroneLaunchState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;
                }

                telemetryHandler.addData("Drone run state", state);
                telemetryHandler.addData("Drone launcher velocity", droneMotor.getVelocity());
            }

            @Override
            public boolean isFinished() {
                return state == DroneLaunchState.IDLE;
            }

            @Override
            public void end(boolean interruptible) {
                state = DroneLaunchState.SPINNING_UP;
            }
        };
    }

    private enum DroneLaunchState {
        SPINNING_UP,
        LAUNCHING,
        IDLE
    }
}
