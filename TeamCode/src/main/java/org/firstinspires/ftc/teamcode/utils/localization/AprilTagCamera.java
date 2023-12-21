package org.firstinspires.ftc.teamcode.utils.localization;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform3d;
import org.firstinspires.ftc.vision.VisionPortal;

public class AprilTagCamera {
    private final CameraName cameraName;
    private final Size resolution;
    private final VisionPortal.StreamFormat streamFormat;
    private final Transform3d robotToCamera;

    public AprilTagCamera(CameraName cameraName, Size resolution, VisionPortal.StreamFormat streamFormat, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.resolution = resolution;
        this.streamFormat = streamFormat;
        this.robotToCamera = robotToCamera;
    }

    public CameraName getCameraName() {
        return cameraName;
    }

    public Size getResolution() {
        return resolution;
    }

    public VisionPortal.StreamFormat getStreamFormat() {
        return streamFormat;
    }

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }
}
