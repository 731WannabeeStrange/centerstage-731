package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher extends SubsystemBase {
    public static double LAUNCH_POS = 0.3;
    public static double ARMED_POS = 1;

    private final Servo drone;

    public DroneLauncher(HardwareMap hardwareMap) {
        drone = hardwareMap.get(Servo.class, "drone");

        drone.setPosition(ARMED_POS);
    }

    public void launchDrone() {
        drone.setPosition(LAUNCH_POS);
    }
}
