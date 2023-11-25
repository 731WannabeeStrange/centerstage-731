package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class VectorMath {

    private VectorMath() {
    }

    public static Vector2d rotate(Vector2d vector, Rotation2d angle) {
        double normalized = angle.log();
        return new Vector2d(
                Math.cos(normalized) * vector.x - Math.sin(normalized) * vector.y,
                Math.sin(normalized) * vector.x + Math.cos(normalized) * vector.y
        );
    }
}
