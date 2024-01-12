package org.firstinspires.ftc.teamcode.utils.old.geometry;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class Vector3d {
    private final VectorF v;

    public Vector3d() {
        v = VectorF.length(3);
    }

    public Vector3d(VectorF vector) {
        if (vector.length() != 3) {
            throw new IllegalArgumentException("Argument to Vector3d constructor needs to be 3D!");
        }
        v = vector;
    }

    public Vector3d(double x, double y, double z) {
        v = new VectorF((float) x, (float) y, (float) z);
    }

    public Vector3d(Vector2d position, double z) {
        v = new VectorF((float) position.x, (float) position.y, (float) z);
    }

    public Vector3d(double distance, Rotation3d angle) {
        Vector3d rectangular = new Vector3d(distance, 0, 0).rotateBy(angle);
        v = new VectorF((float) rectangular.getX(), (float) rectangular.getY(), (float) rectangular.getZ());
    }

    public double getX() {
        return v.get(0);
    }

    public double getY() {
        return v.get(1);
    }

    public double getZ() {
        return v.get(2);
    }

    public Vector3d rotateBy(Rotation3d other) {
        Quaternion p = new Quaternion(0, v.get(0), v.get(1), v.get(2), System.nanoTime());
        Quaternion qprime = other.getQuaternion().multiply(p, p.acquisitionTime).multiply(other.getQuaternion().inverse(), p.acquisitionTime);
        return new Vector3d(qprime.x, qprime.y, qprime.z);
    }

    public Vector2d toVector2d() {
        return new Vector2d(v.get(0), v.get(1));
    }

    public Vector3d plus(Vector3d other) {
        return new Vector3d(v.added(other.v));
    }

    public Vector3d minus(Vector3d other) {
        return new Vector3d(v.subtracted(other.v));
    }

    public Vector3d unaryMinus() {
        return times(-1);
    }

    public Vector3d times(double scalar) {
        return new Vector3d(v.multiplied((float) scalar));
    }

    public Vector3d div(double scalar) {
        return times(1 / scalar);
    }

    public double norm() {
        return v.magnitude();
    }

    public double getDistance(Vector3d other) {
        return Math.sqrt(
                Math.pow(other.v.get(0) - v.get(0), 2) + Math.pow(other.v.get(1) - v.get(1), 2) + Math.pow(other.v.get(2) - v.get(2), 2)
        );
    }
}