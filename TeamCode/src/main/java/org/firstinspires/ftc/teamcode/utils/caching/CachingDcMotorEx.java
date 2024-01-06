package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachingDcMotorEx extends CachingDcMotor implements DcMotorEx {
    private final DcMotorEx dcMotorEx;

    public CachingDcMotorEx(DcMotorEx motor) {
        super(motor);
        this.dcMotorEx = motor;
    }

    public CachingDcMotorEx(DcMotorEx motor, double targetPositionChangeThreshold) {
        super(motor, targetPositionChangeThreshold);
        this.dcMotorEx = motor;
    }

    public CachingDcMotorEx(DcMotorEx motor, double changeThreshold, double targetPositionChangeThreshold) {
        super(motor, changeThreshold, targetPositionChangeThreshold);
        this.dcMotorEx = motor;
    }

    @Override
    public void setMotorEnable() {
        dcMotorEx.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        dcMotorEx.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return dcMotorEx.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        dcMotorEx.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return dcMotorEx.getVelocity();
    }

    @Override
    public void setVelocity(double angularRate) {
        dcMotorEx.setVelocity(angularRate);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return dcMotorEx.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        dcMotorEx.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        dcMotorEx.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        dcMotorEx.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        dcMotorEx.setPositionPIDFCoefficients(p);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return dcMotorEx.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return dcMotorEx.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPositionTolerance() {
        return dcMotorEx.getTargetPositionTolerance();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        dcMotorEx.setTargetPositionTolerance(tolerance);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return dcMotorEx.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return dcMotorEx.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        dcMotorEx.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return dcMotorEx.isOverCurrent();
    }
}
