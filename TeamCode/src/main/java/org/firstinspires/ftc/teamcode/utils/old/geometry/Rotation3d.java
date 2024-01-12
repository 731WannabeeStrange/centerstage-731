package org.firstinspires.ftc.teamcode.utils.old.geometry;

import com.acmerobotics.roadrunner.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class Rotation3d {
    private final Quaternion q;

    public Rotation3d() {
        q = new Quaternion();
    }

    public Rotation3d(Quaternion q) {
        this.q = q.normalized();
    }

    public Rotation3d(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);

        q = new Quaternion(
                (float) (cr * cp * cy + sr * sp * sy),
                (float) (sr * cp * cy - cr * sp * sy),
                (float) (cr * sp * cy + sr * cp * sy),
                (float) (cr * cp * sy - sr * sp * cy),
                System.nanoTime());
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

    public double getRoll() {
        double cxcy = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        double sxcy = 2.0 * (q.w * q.x + q.y * q.z);
        double cy_sq = cxcy * cxcy + sxcy * sxcy;
        if (cy_sq > 1e-20) {
            return Math.atan2(sxcy, cxcy);
        } else {
            return 0.0;
        }
    }

    public double getPitch() {
        double ratio = 2.0 * (q.w * q.y - q.z * q.x);
        if (Math.abs(ratio) >= 1.0) {
            return Math.copySign(Math.PI / 2.0, ratio);
        } else {
            return Math.asin(ratio);
        }
    }

    public double getYaw() {
        double cycz = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double cysz = 2.0 * (q.w * q.z + q.x * q.y);
        double cy_sq = cycz * cycz + cysz * cysz;
        if (cy_sq > 1e-20) {
            return Math.atan2(cysz, cycz);
        } else {
            return Math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z);
        }
    }

    public Rotation2d toRotation2d() {
        return Rotation2d.fromDouble(getYaw());
    }

    public Quaternion getQuaternion() {
        return q;
    }
}