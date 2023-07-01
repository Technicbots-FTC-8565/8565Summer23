package org.firstinspires.ftc.teamcode.localization

import com.acmerobotics.roadrunner.geometry.Vector2d
import kotlin.math.PI

object Field {
    const val DIMENSION = 144.0 // in inches
    const val XM = DIMENSION / 2
    const val YM = DIMENSION / 2
    const val Xm = -DIMENSION / 2
    const val Ym = -DIMENSION / 2
    
    val TR = Vector2d(XM, YM)
    val BR = Vector2d(XM, Ym)
    val BL = Vector2d(Xm, Ym)
    val TL = Vector2d(Xm, YM)

    /**
     * Returns the angles between each of the field corners and the specified point
     * All angles are normed between 0 and 2PI
     */
    fun getAngles(point: Vector2d): Array<Double> {
        return arrayOf(TR.minus(point).angle() % (2 * PI), BR.minus(point).angle() % (2 * PI), BL.minus(point).angle() % (2 * PI), TL.minus(point).angle() % (2 * PI))
    }

    enum class Wall(val x: Double, val y: Double) {
        LEFT(Xm, 0.0),
        RIGHT(XM, 0.0),
        TOP(0.0, YM),
        BOTTOM(0.0, Ym)
    }
}