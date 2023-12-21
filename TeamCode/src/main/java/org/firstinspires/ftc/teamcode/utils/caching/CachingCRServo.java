package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CachingCRServo extends CachingDcMotorSimple implements CRServo {
    private final CRServo servo;

    public CachingCRServo(CRServo servo) {
        this(servo, 0.02);
    }

    public CachingCRServo(CRServo servo, double changeThreshold) {
        super(servo, changeThreshold);
        this.servo = servo;
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }
}
