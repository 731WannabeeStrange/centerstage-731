package org.firstinspires.ftc.teamcode.utils.geometry;

import com.acmerobotics.roadrunner.Rotation2d;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class Rotation3d {
    private final Quaternion q;

    public Rotation3d() {
        q = new Quaternion();
    }

    public Rotation3d(Quaternion q) {
        this.q = q.normalized();
    }

    public Rotation3d(MatrixF rotMatrix) {
        q = Quaternion.fromMatrix(rotMatrix, System.nanoTime());
    }

    public Rotation3d(double roll, double pitch, double yaw) {
        MatrixF rotMatrix = Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ,
                AngleUnit.RADIANS, (float) roll, (float) pitch, (float) yaw);
        q = Quaternion.fromMatrix(rotMatrix, System.nanoTime());
    }

    public Rotation3d(Vector3d axis, double angle) {
        double norm = axis.norm();
        if (norm == 0) {
            q = new Quaternion();
            return;
        }

        Vector3d v = axis.times((float) 1.0 / norm).times((float) Math.sin(angle / 2.0));
        q = new Quaternion((float) Math.cos(angle / 2.0), (float) v.getX(), (float) v.getY(), (float) v.getZ(), System.nanoTime());
    }

    public Rotation3d plus(Rotation3d other) {
        return rotateBy(other);
    }

    public Rotation3d minus(Rotation3d other) {
        return rotateBy(other.unaryMinus());
    }

    public Rotation3d unaryMinus() {
        return new Rotation3d(q.inverse());
    }

    public Rotation3d times(double scalar) {
        if (q.w >= 0) {
            return new Rotation3d(
                    new Vector3d(q.x, q.y, q.z),
                    2.0 * scalar * Math.acos(q.w)
            );
        } else {
            return new Rotation3d(
                    new Vector3d(-q.x, -q.y, -q.z),
                    2.0 * scalar * Math.acos(q.w)
            );
        }
    }

    public Rotation3d div(double scalar) {
        return times(1 / scalar);
    }

    public Rotation3d rotateBy(Rotation3d other) {
        return new Rotation3d(other.q.multiply(q, System.nanoTime()));
    }

    // roll
    public double getX() {
        return q.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
    }

    // pitch
    public double getY() {
        return q.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
    }

    // yaw
    public double getZ() {
        return q.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }

    public Rotation2d toRotation2d() {
        return Rotation2d.fromDouble(getZ());
    }

    public Quaternion getQuaternion() {
        return q;
    }
}
