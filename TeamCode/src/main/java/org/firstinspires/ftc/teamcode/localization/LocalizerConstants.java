package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LocalizerConstants {

    // LOCALIZATION

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE_MULT = 1.02; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -5.5; // in; offset of the lateral wheel
    public static double FORWARD_HORIZONTAL_OFFSET = 1; // in; offset of the lateral wheel
    public static double LATERAL_HORIZONTAL_OFFSET = 0.25; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 0.37;
    public static double Y_MULTIPLIER = -0.37;
}
