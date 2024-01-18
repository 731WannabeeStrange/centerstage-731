package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.Gamepad

class Rumbler(private val gamepad: Gamepad) {
    fun rumble(power1: Double, power2: Double, time: Int) = gamepad.rumble(power1, power2, time)
    fun rumble(time: Int) = gamepad.rumble(time)
    fun rumbleBlips(count: Int) = gamepad.rumbleBlips(count)
    fun stopRumble() = gamepad.stopRumble()
    fun isRumbling() = gamepad.isRumbling
}