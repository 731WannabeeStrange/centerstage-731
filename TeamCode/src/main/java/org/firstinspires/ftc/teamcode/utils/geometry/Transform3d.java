package org.firstinspires.ftc.teamcode.utils.geometry;

public class Transform3d {
    private final Vector3d translation;
    private final Rotation3d rotation;

    public Transform3d(Pose3d initial, Pose3d last) {
        translation = last.getPosition()
                .minus(initial.getPosition())
                .rotateBy(initial.getRotation().unaryMinus());

        rotation = last.getRotation().minus(initial.getRotation());
    }

    public Transform3d(Vector3d translation, Rotation3d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Transform3d(double x, double y, double z, Rotation3d rotation) {
        translation = new Vector3d(x, y, z);
        this.rotation = rotation;
    }

    public Transform3d() {
        translation = new Vector3d();
        rotation = new Rotation3d();
    }

    public Transform3d times(double scalar) {
        return new Transform3d(translation.times(scalar), rotation.times(scalar));
    }

    public Transform3d div(double scalar) {
        return times(1 / scalar);
    }

    public Transform3d plus(Transform3d other) {
        return new Transform3d(new Pose3d(), new Pose3d().transformBy(this).transformBy(other));
    }

    public double getX() {
        return translation.getX();
    }

    public double getY() {
        return translation.getY();
    }

    public double getZ() {
        return translation.getZ();
    }

    public Vector3d getTranslation() {
        return translation;
    }

    public Rotation3d getRotation() {
        return rotation;
    }

    public Transform3d inverse() {
        return new Transform3d(
                translation.unaryMinus().rotateBy(rotation.unaryMinus()),
                rotation.unaryMinus()
        );
    }
}
