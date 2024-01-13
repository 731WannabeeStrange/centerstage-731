package org.firstinspires.ftc.teamcode.utils.old

import android.util.Size

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.teamcode.utils.old.geometry.Transform3d
import org.firstinspires.ftc.vision.VisionPortal

data class AprilTagCamera(
    val cameraName: CameraName,
    val resolution: Size,
    val streamFormat: VisionPortal.StreamFormat,
    val robotToCamera: Transform3d
)