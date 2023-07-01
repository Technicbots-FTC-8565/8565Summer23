package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

import java.util.List;
import java.util.Objects;

@Config
public class Relocalizer extends ExtendedKalmanFilter {
    // Diagonal matrix elements for process noise cov matrix Q
    public static double Q1 = 1;
    public static double Q2 = 1;
    public static double Q3 = 1;

    // Diagonal matrix elements for measurement noise cov matrix R
    public static double R1 = 1;
    public static double R2 = 1;
    public static double R3 = 1;
    private ThreeWheelTrackingLocalizer odometry;
    private Rev2mDistanceDriver distanceDriver;

    public Relocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels, Pose2d initialPose) {
        // construct diagonal Q and R noise cov matrices
        super(new DMatrixRMaj(new double[][] {
                new double[] { Q1, 0, 0 },
                new double[] { 0, Q2, 0 },
                new double[] { 0, 0, Q3 }
        }), new DMatrixRMaj(new double[][] {
                new double[] { R1, 0, 0 },
                new double[] { 0, R2, 0 },
                new double[] { 0, 0, R3 }
        }), poseToMatrix(initialPose));
        this.odometry = new ThreeWheelTrackingLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        this.distanceDriver = new Rev2mDistanceDriver(hardwareMap);
    }

    public static DMatrixRMaj poseToMatrix(Pose2d pose) {
        return new DMatrixRMaj(new double[] { pose.getX(), pose.getY(), pose.getHeading() });
    }

    public static Pose2d matrixToPose(DMatrixRMaj matrix) {
        return new Pose2d(matrix.data[0], matrix.data[1], matrix.data[2]);
    }
    /**
     * describes how the state evolves
     * just going to use the regular odometry prediction for the state update
     * @param u input vector
     * @return delta vector for state estimate based on u and current state estimate
     */
    @Override
    public SimpleMatrix f(DMatrixRMaj u) {
        odometry.update();
        return new SimpleMatrix(poseToMatrix(Objects.requireNonNull(odometry.getPoseVelocity())));
    }

    /**
     * @param u input vector
     * @return the jacobian of {@link #f} with respect to the state vector
     */
    @Override
    public SimpleMatrix F(DMatrixRMaj u) {
        return null;
    }

    /**
     * Pretty much describes what the measurement should look like given the current state
     *
     * @return innovation estimate based on measurement vector
     */
    @Override
    public SimpleMatrix h() {
        return null;
    }

    /**
     * @return the jacobian of {@link #h} with respect to the state vector
     */
    @Override
    public SimpleMatrix H() {
        return null;
    }

    public void update() {
        super.iterate(new DMatrixRMaj((double[][]) Objects.requireNonNull(odometry.getWheelVelocities().toArray())), new DMatrixRMaj((double[][]) distanceDriver.getDistances().toArray()));
    }
}
