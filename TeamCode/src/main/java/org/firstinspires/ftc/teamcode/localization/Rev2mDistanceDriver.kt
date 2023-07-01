package org.firstinspires.ftc.teamcode.localization

import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Rev2mDistanceDriver(hardwareMap: HardwareMap) {
    private val dF = hardwareMap.get(Rev2mDistanceSensor::class.java, "distF")
    private val dR = hardwareMap.get(Rev2mDistanceSensor::class.java, "distR");
    private val dL = hardwareMap.get(Rev2mDistanceSensor::class.java, "distL");

    fun getDistances(): List<Double> {
        return listOf(dF.getDistance(DistanceUnit.INCH), dR.getDistance(DistanceUnit.INCH), dL.getDistance(DistanceUnit.INCH))
    }

    companion object {
        /**
         * The time it takes for the distance sensors to update, in ms
         */
        @JvmStatic
        val CACHE_TIME = 33
    }
}