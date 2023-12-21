package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class CachingDcMotor extends CachingDcMotorSimple implements DcMotor {
    private double cachedTargetPosition = 0;
    private final double targetPositionChangeThreshold;
    private final DcMotor dcMotor;

    public CachingDcMotor(DcMotor motor) {
        this(motor, 10);
    }

    public CachingDcMotor(DcMotor motor, double targetPositionChangeThreshold) {
        super(motor);
        this.dcMotor = motor;
        this.targetPositionChangeThreshold = targetPositionChangeThreshold;
    }

    public CachingDcMotor(DcMotor motor, double changeThreshold, double targetPositionChangeThreshold) {
        super(motor, changeThreshold);
        this.dcMotor = motor;
        this.targetPositionChangeThreshold = targetPositionChangeThreshold;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return dcMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        dcMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return dcMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return dcMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotor.getZeroPowerBehavior();
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        dcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        if (Math.abs(cachedTargetPosition - position) >= targetPositionChangeThreshold) {
            dcMotor.setTargetPosition(position);
            cachedTargetPosition = position;
        }
    }

    @Override
    public int getTargetPosition() {
        return dcMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return dcMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return dcMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return dcMotor.getMode();
    }
}
