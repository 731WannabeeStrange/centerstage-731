package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public abstract class CachingHardwareDevice implements HardwareDevice {
    private final HardwareDevice hardwareDevice;

    public CachingHardwareDevice(HardwareDevice hardwareDevice) {
        this.hardwareDevice = hardwareDevice;
    }

    @Override
    public Manufacturer getManufacturer() {
        return hardwareDevice.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return hardwareDevice.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return hardwareDevice.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return hardwareDevice.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        hardwareDevice.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        hardwareDevice.close();
    }
}
