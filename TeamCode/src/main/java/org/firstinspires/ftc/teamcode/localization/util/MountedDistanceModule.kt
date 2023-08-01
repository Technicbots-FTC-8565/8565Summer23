package org.firstinspires.ftc.teamcode.localization.util

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.ejml.simple.SimpleMatrix

class MountedDistanceModule(hardwareMap: HardwareMap) {
    // TODO: tune these poses for an actual robot
    private val dL = MountedDistanceSensor(hardwareMap.get(Rev2mDistanceSensor::class.java, "distL"), Pose2d())
    private val dR = MountedDistanceSensor(hardwareMap.get(Rev2mDistanceSensor::class.java, "distR"), Pose2d())
    private val dF = MountedDistanceSensor(hardwareMap.get(Rev2mDistanceSensor::class.java, "distF"), Pose2d())

    fun draw(canvas: Canvas, robotPose: Pose2d) {
        dL.draw(canvas, robotPose)
        dR.draw(canvas, robotPose)
        dF.draw(canvas, robotPose)
    }

    fun getDistances(): DoubleArray {
        return doubleArrayOf(dL.getDistance(), dR.getDistance(), dF.getDistance())
    }

    fun getPredictedDistances(robotPose: Pose2d): DoubleArray {
        return doubleArrayOf(dL.getPredictedDistance(robotPose), dR.getPredictedDistance(robotPose), dF.getPredictedDistance(robotPose))
    }

    /**
     * @return The jacobian of the sensor model w.r.t the incoming robot pose (state)
     */
    fun jacobian(robotPose: Pose2d): SimpleMatrix {
        return SimpleMatrix(arrayOf(
            dL.partialDerivative(robotPose),
            dR.partialDerivative(robotPose),
            dF.partialDerivative(robotPose)
        ))
    }

    companion object {
        /**
         * The time it takes for the distance sensors to update, in ms
         */
        @JvmStatic
        val CACHE_TIME = 33
    }
}