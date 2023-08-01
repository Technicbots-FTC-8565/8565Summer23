package org.firstinspires.ftc.teamcode.localization.util

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.tan

@Config
class MountedDistanceSensor(private val sensor: Rev2mDistanceSensor, mountingPose: Pose2d) {
    val pose = mountingPose
    val lowPassFilter: LowPassFilter = LowPassFilter(LPF_ALPHA)

    fun draw(canvas: Canvas, robotPose: Pose2d) {
        // canvas.setStrokeWidth(STROKE_WIDTH)
        val distance = getDistance()
        if (distance > MAX_DIST - 2) canvas.setStroke("#db1414")
        else canvas.setStroke("#87f542")
        val p = getActualPose(robotPose)
        canvas.strokeLine(p.x, p.y, p.x + p.headingVec().x * distance, p.y + p.headingVec().y * distance)
    }

    fun getActualPose(robotPose: Pose2d): Pose2d {
        val x = this.pose.x
        val y = this.pose.y
        val angle = robotPose.heading

        // Apply a simple rotation to rotate the current point about the robot, then add on the robot's pose
        return Pose2d(x * cos(angle) - y * sin(angle), y * cos(angle) + x * sin(angle), (pose.heading + robotPose.heading) % (2 * PI)).plus(Pose2d(robotPose.x, robotPose.y, 0.0))
    }

    /**
     * @return The wall this distance sensor is currently facing
     */
    fun getFacingWall(robotPose: Pose2d): Field.Wall {
        val angles = Field.getAngles(this.getActualPose(robotPose).vec())
        val heading = this.getActualPose(robotPose).heading
        if (isBetween(angles[1], angles[0], heading)) return Field.Wall.RIGHT
        else if (isBetween(angles[2], angles[1], heading)) return Field.Wall.BOTTOM
        else if (isBetween(angles[3], angles[2], heading)) return Field.Wall.LEFT
        return Field.Wall.TOP
    }

    /**
     * Distance reading of the sensor, in inches
     */
    fun getDistance(): Double {
        return Range.clip(lowPassFilter.update(this.sensor.getDistance(DistanceUnit.INCH)), MIN_DIST, MAX_DIST)
    }

    /**
     * Partial derivative of the sensor model w.r.t. the incoming robot pose (state)
     */
    fun partialDerivative(robotPose: Pose2d): DoubleArray {
        val wall = getFacingWall(robotPose)
        val currPose = this.getActualPose(robotPose)
        val x = currPose.x
        val y = currPose.y
        val theta = currPose.heading // theta should always be between 0 and 2pi rad

        return when (wall) {
            Field.Wall.TOP -> {
                doubleArrayOf(0.0, 0.0, Range.clip((y - wall.y) * cos(theta) / sin(PI / 2 - theta).pow(2), MIN_DIST, MAX_DIST))
            }
            Field.Wall.RIGHT -> {
                doubleArrayOf(0.0, 0.0, Range.clip((wall.x - x) / cos(theta) * tan(theta), MIN_DIST, MAX_DIST))
            }
            Field.Wall.LEFT -> {
                doubleArrayOf(0.0, 0.0, Range.clip((x - wall.x) / cos(theta) * tan(theta), MIN_DIST, MAX_DIST))
            }
            else -> {
                doubleArrayOf(0.0, 0.0, Range.clip((y - wall.y) / sin(theta) / tan(theta), MIN_DIST, MAX_DIST))
            }
        }
    }

    fun getPredictedDistance(robotPose: Pose2d): Double {
        val wall = getFacingWall(robotPose)
        val currPose = this.getActualPose(robotPose)
        val x = currPose.x
        val y = currPose.y
        val theta = currPose.heading // theta should always be between 0 and 2pi rad
        
        return when (wall) {
            Field.Wall.TOP -> Range.clip((wall.y - y) / cos(PI / 2 - theta), MIN_DIST, MAX_DIST)
            Field.Wall.RIGHT -> Range.clip((wall.x - x) / cos(theta), MIN_DIST, MAX_DIST)
            Field.Wall.LEFT -> Range.clip((x - wall.x) / cos(theta - PI), MIN_DIST, MAX_DIST)
            else -> Range.clip((y - wall.y) / cos(3 * PI / 2 - theta), MIN_DIST, MAX_DIST)
        }
    }

    companion object {
        @JvmStatic
        val MIN_DIST = 0.0
        /** Maximum distance reading of the sensor, in inches (equivalent to 2m) */
        @JvmStatic
        val MAX_DIST = 78.7401575
        /** Stroke width of the rendered distance lines on dashboard */
        @JvmStatic
        val STROKE_WIDTH = 3
        /** Low pass filter alpha coefficient (higher = greater weight to recent values) */
        @JvmStatic
        val LPF_ALPHA = 0.7

        /**
         * Checks to see if an angle is between two other angles, with all angles on a 0 to 2pi rad range and starting from the x-axis going ccw
         */
        private fun isBetween(start: Double, end: Double, angle: Double): Boolean {
            val clampedEnd = if ((end - start) < 0.0) end - start + 2 * PI  else  end - start
            val clampedActual = if ((angle - start) < 0.0) angle - start + 2 * PI else  angle - start
            return clampedEnd < clampedActual
        }
    }
}