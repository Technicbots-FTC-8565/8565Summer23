package org.firstinspires.ftc.teamcode.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap

class MountedDistanceModule(hardwareMap: HardwareMap) {
    // TODO: tune these poses for an actual robot
    private val dL = MountedDistanceSensor(hardwareMap.get(Rev2mDistanceSensor::class.java, "distL"), Pose2d())
    private val dR = MountedDistanceSensor(hardwareMap.get(Rev2mDistanceSensor::class.java, "distR"), Pose2d())
    private val dF = MountedDistanceSensor(hardwareMap.get(Rev2mDistanceSensor::class.java, "distF"), Pose2d())

    fun getDistances(): Array<Double> {
        return arrayOf(dL.getDistance(), dR.getDistance(), dF.getDistance())
    }

    fun getPredictedDistances(robotPose: Pose2d): Array<Double> {
        return arrayOf(dL.getPredictedDistance(robotPose), dR.getPredictedDistance(robotPose), dF.getPredictedDistance(robotPose))
    }

    companion object {
        /**
         * The time it takes for the distance sensors to update, in ms
         */
        @JvmStatic
        val CACHE_TIME = 33
    }
}