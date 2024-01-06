package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.button.Trigger
import java.util.function.Supplier

class HotZoneTrigger(
    private val poseSupplier: Supplier<Pose2d>,
    private val maximum: Vector2d,
    private val minimum: Vector2d
) : Trigger() {
    override fun get(): Boolean {
        val currentPosition = poseSupplier.get().position
        return currentPosition.x > minimum.x && currentPosition.x < maximum.x &&
                currentPosition.y > minimum.y && currentPosition.y < maximum.y
    }
}
