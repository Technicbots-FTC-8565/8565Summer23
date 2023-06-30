package org.firstinspires.ftc.teamcode.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder
import kotlin.math.*
import java.util.Arrays

/*
* Sample tracking wheel localizer implementation assuming the standard configuration:
*
*    /--------------\
*    |     ____     |
*    |     ----     |
*    | ||        || |
*    | ||        || |
*    |              |
*    |              |
*    \--------------/
*
*/
@Config
class ThreeWheelTrackingLocalizer(
    hardwareMap: HardwareMap,
    private val lastEncPositions: MutableList<Int>,
    private val lastEncVels: MutableList<Int>
) :
    ThreeTrackingWheelLocalizer(
        Arrays.asList(
            Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // left
            Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),  // right
            Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
        )
    ) {
    private val frontEncoder: Encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "rightBack"))
    private val rightEncoder: Encoder =
        Encoder(hardwareMap.get(DcMotorEx::class.java, "rightFront"))
    private val leftEncoder: Encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "leftBack"))

    init {
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    fun setInitialPoseEstimate(pose: Pose2d) {
        poseEstimate
    }

    override fun getWheelPositions(): List<Double> {
        val leftPos = leftEncoder.currentPosition
        val rightPos = rightEncoder.currentPosition
        val frontPos = frontEncoder.currentPosition
        lastEncPositions.clear()
        lastEncPositions.add(leftPos)
        lastEncPositions.add(rightPos)
        lastEncPositions.add(frontPos)
        return Arrays.asList(
            encoderTicksToInches(leftPos.toDouble()),
            encoderTicksToInches(rightPos.toDouble()),
            encoderTicksToInches(frontPos.toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double> {
        val leftVel = leftEncoder.correctedVelocity.toInt()
        val rightVel = rightEncoder.correctedVelocity.toInt()
        val frontVel = frontEncoder.correctedVelocity.toInt()
        lastEncVels.clear()
        lastEncVels.add(leftVel)
        lastEncVels.add(rightVel)
        lastEncVels.add(frontVel)
        return Arrays.asList(
            encoderTicksToInches(leftVel.toDouble()),
            encoderTicksToInches(rightVel.toDouble()),
            encoderTicksToInches(frontVel.toDouble())
        )
    }

    fun setPoseEstimate(estimate: Pose2d) {
        this.poseEstimate = poseEstimate
    }

    /**
     * Converts the pose delta from the current wheel velocities
     */
    fun calculatePoseDelta(): Pose2d {
        val u = getWheelVelocities()
        // robot centric velocities
        val dh = (u[0] - u[1]) / LATERAL_DISTANCE
        val dx = (u[0] + u[1]) / 2
        val dy = u[3] - (FORWARD_OFFSET * dh)
        val theta = poseEstimate.heading
        // apply simple rotation to transform robot centric velocities into field centric velocities
        return Pose2d(dx * cos(theta) - dy * sin(theta), dx * sin(theta) + dy * cos(theta), dh)
    }

    // TODO: See if FTC Dashboard recognizes these
    companion object {
        @JvmStatic
        var TICKS_PER_REV = 0.0

        @JvmStatic
        var WHEEL_RADIUS = 2.0 // in

        @JvmStatic
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed

        @JvmStatic
        var LATERAL_DISTANCE = 10.0 // in; distance between the left and right wheels

        @JvmStatic
        var FORWARD_OFFSET = 4.0 // in; offset of the lateral wheel

        @JvmStatic
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}