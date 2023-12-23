package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CachingDcMotorSimple extends CachingHardwareDevice implements DcMotorSimple {
    private double cachedPower = 0;
    private double changeThreshold;
    private final DcMotorSimple dcMotorSimple;

    public CachingDcMotorSimple(DcMotorSimple motor) {
        this(motor, 0);
    }

    public CachingDcMotorSimple(DcMotorSimple motor, double changeThreshold) {
        super(motor);
        this.dcMotorSimple = motor;
        this.changeThreshold = changeThreshold;
    }

    public double getChangeThreshold() {
        return changeThreshold;
    }

    public void setChangeThreshold(double changeThreshold) {
        this.changeThreshold = changeThreshold;
    }

    @Override
    public void setDirection(Direction direction) {
        dcMotorSimple.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return dcMotorSimple.getDirection();
    }

    @Override
    public void setPower(double power) {
        if (Math.abs(cachedPower - power) >= changeThreshold ||
                (power == 0 && !(cachedPower == 0) || (power >= 1.0 && !(cachedPower >= 1.0) ||
                        (power <= -1.0 && !(cachedPower <= -1.0))))) {
            this.cachedPower = power;
            dcMotorSimple.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return dcMotorSimple.getPower();
    }
}
