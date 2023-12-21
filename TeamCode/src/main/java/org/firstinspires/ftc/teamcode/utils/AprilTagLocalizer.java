package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Optional;

public class AprilTagLocalizer implements AbsoluteLocalizer {
    private final ArrayList<VisionPortal> visionPortals = new ArrayList<>();
    private final ArrayList<AprilTagCamera> cameras = new ArrayList<>();
    private final ArrayList<AprilTagProcessor> aprilTagProcessors = new ArrayList<>();

    private Pose3d referencePose = null;

    // will probably try other strategies
    public enum PoseEstimationStrategy {
        CLOSEST_TO_REFERENCE_POSE,
        CLOSEST_TO_CAMERA_HEIGHT
    }

    private final PoseEstimationStrategy poseEstimationStrategy;

    public AprilTagLocalizer(PoseEstimationStrategy poseEstimationStrategy, AprilTagCamera... cameras) {
        // add appropriate AprilTag options
        AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary());

        if (cameras.length == 1) {
            this.cameras.add(cameras[0]);
            AprilTagProcessor aprilTagProcessor = aprilTagProcessorBuilder.build();
            aprilTagProcessors.add(aprilTagProcessor);
            visionPortals.add(new VisionPortal.Builder()
                    .setCamera(cameras[0].getCameraName())
                    .addProcessor(aprilTagProcessor)
                    .setCameraResolution(cameras[0].getResolution())
                    .setStreamFormat(cameras[0].getStreamFormat())
                    .enableLiveView(true)
                    .build());
        } else {
            // create multi-portal thingy
            int[] portalIds = VisionPortal.makeMultiPortalView(cameras.length, VisionPortal.MultiPortalLayout.HORIZONTAL);

            // iterate over cameras
            for (int i = 0; i < cameras.length; i++) {
                this.cameras.add(cameras[i]);
                AprilTagProcessor aprilTagProcessor = aprilTagProcessorBuilder.build();
                aprilTagProcessors.add(aprilTagProcessor);
                visionPortals.add(new VisionPortal.Builder()
                        .setCamera(cameras[i].getCameraName())
                        .addProcessor(aprilTagProcessor)
                        .setCameraResolution(cameras[i].getResolution())
                        .setStreamFormat(cameras[i].getStreamFormat())
                        .setLiveViewContainerId(portalIds[i])
                        .build());
            }
        }

        this.poseEstimationStrategy = poseEstimationStrategy;
    }

    public Pose2d getReferencePose() {
        return referencePose.toPose2d();
    }

    public void setReferencePose(Pose2d referencePose) {
        this.referencePose = new Pose3d(referencePose);
    }

    @Override
    public Optional<Pose2d> getPoseEstimate() {
        Optional<Pose2d> poseEstimate = Optional.empty();

        switch (poseEstimationStrategy) {
            case CLOSEST_TO_REFERENCE_POSE:
                double smallestPoseDelta = Double.MAX_VALUE;

                // loop over all cameras
                for (int i = 0; i < cameras.size(); i++) {
                    AprilTagCamera camera = cameras.get(i);
                    AprilTagProcessor aprilTagProcessor = aprilTagProcessors.get(i);
                    // loop over each detection in the processor
                    // could change the estimation strategy here
                    for (AprilTagDetection aprilTagDetection : aprilTagProcessor.getDetections()) {
                        if (aprilTagDetection.metadata != null) {
                            // create a Pose3d with where the AprilTag is on the field
                            Pose3d targetPose = new Pose3d(
                                    new Vector3d(aprilTagDetection.metadata.fieldPosition),
                                    new Rotation3d(aprilTagDetection.metadata.fieldOrientation));

                            // create a Transform3d from the camera to the AprilTag
                            AprilTagPoseRaw rawPose = aprilTagDetection.rawPose;
                            Transform3d cameraToTarget = new Transform3d(rawPose.x, rawPose.y, rawPose.z,
                                    new Rotation3d(rawPose.R));

                            // find the estimated robot pose by doing two inverse transformations
                            Pose3d robotPose = targetPose
                                    .transformBy(cameraToTarget.inverse())
                                    .transformBy(camera.getRobotToCamera().inverse());

                            // check if this is the minimum distance from the reference pose
                            double poseDelta = robotPose.getPosition().getDistance(referencePose.getPosition());
                            if (poseDelta < smallestPoseDelta) {
                                smallestPoseDelta = poseDelta;
                                poseEstimate = Optional.of(robotPose.toPose2d());
                            }
                        } // if null, it's weird and there should probably be reporting here
                    }
                }
                break;
            case CLOSEST_TO_CAMERA_HEIGHT:
                double smallestHeightDifference = Double.MAX_VALUE;

                // loop over all cameras
                for (int i = 0; i < cameras.size(); i++) {
                    AprilTagCamera camera = cameras.get(i);
                    AprilTagProcessor aprilTagProcessor = aprilTagProcessors.get(i);
                    // loop over each detection in the processor
                    // could change the estimation strategy here
                    for (AprilTagDetection aprilTagDetection : aprilTagProcessor.getDetections()) {
                        if (aprilTagDetection.metadata != null) {
                            // create a Pose3d with where the AprilTag is on the field
                            Pose3d targetPose = new Pose3d(
                                    new Vector3d(aprilTagDetection.metadata.fieldPosition),
                                    new Rotation3d(aprilTagDetection.metadata.fieldOrientation));

                            // create a Transform3d from the camera to the AprilTag
                            AprilTagPoseRaw rawPose = aprilTagDetection.rawPose;
                            Transform3d cameraToTarget = new Transform3d(rawPose.x, rawPose.y, rawPose.z,
                                    new Rotation3d(rawPose.R));

                            // find the estimated robot pose by doing two inverse transformations
                            Pose3d robotPose = targetPose
                                    .transformBy(cameraToTarget.inverse())
                                    .transformBy(camera.getRobotToCamera().inverse());

                            // check if this is the minimum height difference from the camera
                            double heightDifference = Math.abs(
                                    camera.getRobotToCamera().getZ() -
                                            targetPose
                                                    .transformBy(cameraToTarget.inverse())
                                                    .getPosition()
                                                    .getZ()
                            );
                            if (heightDifference < smallestHeightDifference) {
                                smallestHeightDifference = heightDifference;
                                poseEstimate = Optional.of(robotPose.toPose2d());
                            }
                        } // if null, it's weird and there should probably be reporting here
                    }
                }
                break;
        }

        return poseEstimate;
    }
}
