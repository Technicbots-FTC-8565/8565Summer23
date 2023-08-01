package org.firstinspires.ftc.teamcode.localization.util

import com.qualcomm.robotcore.util.Range

/**
 * Simple low-pass Exponential Moving Average (EMA) filter for noise reduction.
 * Attenuates high frequencies and hopefully prevents distance sensor noise from messing up the relocalizer.
 */
class LowPassFilter(initAlpha: Double) {
    var alpha = Range.clip(initAlpha, 0.0, 1.0)
    var out: Double = 0.0

    fun setAlpha(newAlpha: Double) {
        this.alpha = Range.clip(newAlpha, 0.0, 1.0)
    }

    fun update(inp: Double): Double {
        this.out = this.alpha * inp + (1.0 - this.alpha) * this.out
        return this.out
    }
}