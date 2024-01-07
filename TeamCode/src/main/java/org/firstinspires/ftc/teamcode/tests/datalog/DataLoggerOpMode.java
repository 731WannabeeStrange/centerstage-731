package org.firstinspires.ftc.teamcode.tests.datalog;

import android.util.Size;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp
public class DataLoggerOpMode extends LinearOpMode {
    private static final String LOG_FILE_NAME = "datalog.txt";

    @Override
    public void runOpMode() throws InterruptedException {
        BufferedFileWriter bufferedFileWriter = new BufferedFileWriter(String.format("/sdcard/FIRST/Datalogs/%s.txt", LOG_FILE_NAME));

        MecanumDrive.Params PARAMS = new MecanumDrive.Params();

        SimplifiedMecanumDrive drive = new SimplifiedMecanumDrive(hardwareMap);
        IncrementalLocalizer localizer = new IncrementalThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);

        AprilTagProcessor aprilTagProcessor1 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        AprilTagProcessor aprilTagProcessor2 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        int[] portalIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor1)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(800, 600))
                .setLiveViewContainerId(portalIds[0])
                .build();

        VisionPortal visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTagProcessor1)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(800, 600))
                .setLiveViewContainerId(portalIds[1])
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            try {
                bufferedFileWriter.writeMessage(new LogMessage(
                        localizer.update(),
                        aprilTagProcessor1.getDetections(),
                        aprilTagProcessor2.getDetections()
                ));
            } catch (IOException e) {
                e.printStackTrace();
                throw new RuntimeException("Unable to write log message");
            }
        }

        try {
            bufferedFileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Unable to close bufferedFileWriter");
        }
    }

    private class LogMessage {
        private final long timestamp;
        private final Twist2dDual<Time> localizerTwist;
        private final ArrayList<AprilTagDetection> camera1Detections;
        private final ArrayList<AprilTagDetection> camera2Detections;

        public LogMessage(Twist2dDual<Time> localizerTwist, ArrayList<AprilTagDetection> camera1Detections, ArrayList<AprilTagDetection> camera2Detections) {
            timestamp = System.nanoTime();
            this.localizerTwist = localizerTwist;
            this.camera1Detections = camera1Detections;
            this.camera2Detections = camera2Detections;
        }
    }

    private class BufferedFileWriter {
        private final BufferedWriter bufferedWriter;
        private final Gson gson = new Gson();

        public BufferedFileWriter(String filePath) {
            File tmp = new File(filePath);
            if (!tmp.exists()) {
                tmp.getParentFile().mkdirs();
            }
            try {
                bufferedWriter = new BufferedWriter(new FileWriter(filePath, false));
            } catch (IOException e) {
                e.printStackTrace();
                throw new RuntimeException("Unable to create output file handle");
            }
        }

        public void writeMessage(LogMessage message) throws IOException {
            bufferedWriter.write(gson.toJson(message));
        }

        public void close() throws IOException {
            bufferedWriter.close();
        }
    }
}
