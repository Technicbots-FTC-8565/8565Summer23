package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

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

    public Relocalizer(Pose2d initialPose) {
        super(new DMatrixRMaj(), new DMatrixRMaj(), new DMatrixRMaj());
    }

    public static DMatrixRMaj poseToMatrix(Pose2d pose) {
        return new DMatrixRMaj();
    }
    /**
     * describes how the state evolves
     *
     * @param u input vector
     * @return delta vector for state estimate based on u and current state estimate
     */
    @Override
    public SimpleMatrix f(DMatrixRMaj u) {
        return null;
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
}
