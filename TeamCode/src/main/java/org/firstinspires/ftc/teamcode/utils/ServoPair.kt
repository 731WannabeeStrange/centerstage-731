package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.Servo

class ServoPair(private val servo1: Servo, private val servo2: Servo) {
    fun setPosition(pos: Double) {
        servo1.position = pos
        servo2.position = 1 - pos
    }
}