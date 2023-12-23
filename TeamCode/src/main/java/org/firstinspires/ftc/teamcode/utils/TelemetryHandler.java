package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class TelemetryHandler {
    private final Telemetry systemTelemetry;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    public TelemetryHandler(Telemetry systemTelemetry) {
        this.systemTelemetry = systemTelemetry;
    }

    public void addData(String key, Object value) {
        packet.put(key, value);
        systemTelemetry.addData(key, value);
    }

    public void addAll(Map<String, Object> map) {
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            packet.put(entry.getKey(), entry.getValue());
            systemTelemetry.addData(entry.getKey(), entry.getValue());
        }
    }

    public Canvas fieldOverlay() {
        return packet.fieldOverlay();
    }

    public void update() {
        dashboard.sendTelemetryPacket(packet);
        systemTelemetry.update();
        packet = new TelemetryPacket();
    }
}
