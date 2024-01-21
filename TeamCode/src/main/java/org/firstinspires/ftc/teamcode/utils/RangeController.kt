package org.firstinspires.ftc.teamcode.utils

class RangeController {
    companion object {
        @JvmStatic
        fun getPositionFromRange(fraction: Double, min: Double, max: Double): Double {
            val clampedFraction = Math.max(0.0, Math.min(1.0, fraction))
            val offset = (max - min) * clampedFraction
            return min + offset
        }
    }
}
