package org.firstinspires.ftc.teamcode.utils.geometry;

import com.acmerobotics.roadrunner.Pose2d;

public class Pose3d {
    private final Vector3d position;
    private final Rotation3d rotation;

    public Pose3d() {
        position = new Vector3d();
        rotation = new Rotation3d();
    }

    public Pose3d(Vector3d position, Rotation3d rotation) {
        this.position = position;
        this.rotation = rotation;
    }

    public Pose3d(double x, double y, double z, Rotation3d rotation) {
        position = new Vector3d(x, y, z);
        this.rotation = rotation;
    }

    public Pose3d(Pose2d pose) {
        position = new Vector3d(pose.position, 0);
        rotation = new Rotation3d(0, 0, pose.heading.toDouble());
    }

    public Vector3d getPosition() {
        return position;
    }

    public Rotation3d getRotation() {
        return rotation;
    }

    public Pose3d times(double scalar) {
        return new Pose3d(position.times(scalar), rotation.times(scalar));
    }

    public Pose3d div(double scalar) {
        return times(1 / scalar);
    }

    public Pose3d transformBy(Transform3d transform) {
        return new Pose3d(
                position.plus(transform.getTranslation().rotateBy(rotation)),
                transform.getRotation().plus(rotation)
        );
    }

    public Pose3d relativeTo(Pose3d other) {
        Transform3d transform = new Transform3d(other, this);
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    public Pose3d rotateBy(Rotation3d angle) {
        return new Pose3d(position.rotateBy(angle), rotation.rotateBy(angle));
    }

    public Pose2d toPose2d() {
        return new Pose2d(position.toVector2d(), rotation.toRotation2d());
    }
}
