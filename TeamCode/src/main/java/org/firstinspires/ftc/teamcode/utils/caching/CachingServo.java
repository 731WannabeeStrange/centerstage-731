package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CachingServo extends CachingHardwareDevice implements Servo {
    private double cachedPosition = 0;
    private double changeThreshold;
    private final Servo servo;

    public CachingServo(Servo servo) {
        this(servo, 0.01);
    }

    public CachingServo(Servo servo, double changeThreshold) {
        super(servo);
        this.changeThreshold = changeThreshold;
        this.servo = servo;
    }

    public double getChangeThreshold() {
        return changeThreshold;
    }

    public void setChangeThreshold(double changeThreshold) {
        this.changeThreshold = changeThreshold;
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        if (Math.abs(cachedPosition - position) >= changeThreshold ||
                (position <= 0.0 && !(cachedPosition <= 0.0)) ||
                (position >= 1.0 && !(cachedPosition >= 1.0))) {
            servo.setPosition(position);
            cachedPosition = position;
        }
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }
}
