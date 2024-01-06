package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.robotcore.external.Telemetry

class TelemetryHandler @JvmOverloads constructor(
    private val systemTelemetry: Telemetry? = null,
) {
    private val dashboard: FtcDashboard = FtcDashboard.getInstance()
    private var packet: TelemetryPacket = TelemetryPacket()

    fun addData(key: String, value: Any) {
        packet.put(key, value)
        systemTelemetry?.addData(key, value)
    }

    fun addAll(map: Map<String, Any>) {
        for ((caption, value) in map) {
            addData(caption, value)
        }
    }

    fun fieldOverlay(): Canvas = packet.fieldOverlay()

    fun update() {
        dashboard.sendTelemetryPacket(packet)
        systemTelemetry?.update()
        packet = TelemetryPacket()
    }
}
