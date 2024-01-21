package org.firstinspires.ftc.teamcode.utils

class RangeController {
    companion object {
        @JvmStatic
        fun clamp(number: Double, min: Double, max: Double) =
            min.coerceAtLeast(max.coerceAtMost(number))

        @JvmStatic
        fun getPositionFromRange(fraction: Double, min: Double, max: Double): Double {
            val clampedFraction = clamp(fraction, 0.0, 1.0)
            val offset = (max - min) * clampedFraction
            return min + offset
        }
    }
}
