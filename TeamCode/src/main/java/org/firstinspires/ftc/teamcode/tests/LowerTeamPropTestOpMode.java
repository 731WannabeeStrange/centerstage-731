package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;
import org.firstinspires.ftc.teamcode.utils.LowerTeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class LowerTeamPropTestOpMode extends LinearOpMode {
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        LowerTeamPropProcessor teamPropProcessor = new LowerTeamPropProcessor();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(1280, 720))
                .build();

        while (opModeInInit()) {
            telemetryHandler.addData("Selection", teamPropProcessor.getSelection());
            telemetryHandler.update();
        }
    }
}
