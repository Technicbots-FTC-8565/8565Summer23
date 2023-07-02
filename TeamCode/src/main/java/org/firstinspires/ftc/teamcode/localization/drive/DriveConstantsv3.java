package org.firstinspires.ftc.teamcode.localization.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */

//TODO: Tune when new chassis comes out, this is ug stuff
@Config
public class DriveConstantsv3 {


    //gobila 5202 0002 0014
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = false;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 10.25);

    public static double WHEEL_RADIUS = 1.89; // in
    public static double GEAR_RATIO = 0.98; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 15; //12/20 12.43v


    public static double kV = 0.017; //1.0 / rpmToVelocity(MAX_RPM);
    //12.46
    public static double kA = 0.001;// 0.0022;
    public static double kStatic = 0.05;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = 3;//Math.toRadians(334); //Math.toRadians(279.3946531);
    public static double MAX_ANG_ACCEL = 4; //Math.toRadians(1100);

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}