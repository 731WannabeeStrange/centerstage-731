package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class TelemetryHandler {
    private static TelemetryHandler instance = null;

    public static synchronized TelemetryHandler getInstance() {
        if (instance == null) {
            instance = new TelemetryHandler();
        }
        return instance;
    }

    private TelemetryHandler() {
    }

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    public TelemetryPacket getCurrentPacket() {
        return packet;
    }

    public void updateCurrentPacket(TelemetryPacket newPacket) {
        packet = newPacket;
    }

    public void sendCurrentPacket() {
        dashboard.sendTelemetryPacket(packet);
        reset();
    }

    public void reset() {
        packet = new TelemetryPacket();
    }
}
