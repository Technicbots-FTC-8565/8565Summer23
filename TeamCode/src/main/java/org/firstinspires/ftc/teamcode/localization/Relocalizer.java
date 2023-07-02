package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.lang3.ArrayUtils;
import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.localization.drive.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.localization.util.MountedDistanceModule;

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
    private MountedDistanceModule distanceModule;

    public Relocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // construct diagonal Q and R noise cov matrices
        super(new DMatrixRMaj(new double[][] {
                new double[] { Q1, 0, 0 },
                new double[] { 0, Q2, 0 },
                new double[] { 0, 0, Q3 }
        }), new DMatrixRMaj(new double[][] {
                new double[] { R1, 0, 0 },
                new double[] { 0, R2, 0 },
                new double[] { 0, 0, R3 }
        }), poseToMatrix(initialPose), MountedDistanceModule.getCACHE_TIME());
        this.odometry = new ThreeWheelTrackingLocalizer(hardwareMap);
        this.distanceModule = new MountedDistanceModule(hardwareMap);
    }

    public static DMatrixRMaj poseToMatrix(Pose2d pose) {
        return new DMatrixRMaj(new double[] { pose.getX(), pose.getY(), pose.getHeading() });
    }

    public static Pose2d matrixToPose(SimpleMatrix matrix) {
        return new Pose2d(matrix.get(0), matrix.get(1), matrix.get(2));
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
        double[] data = u.getData();
        double deltaX = (data[0] + data[1]) / 2;
        double deltaTheta = (data[0] - data[1]) / ThreeWheelTrackingLocalizer.LATERAL_DISTANCE;
        double deltaY = data[2] - ThreeWheelTrackingLocalizer.FORWARD_OFFSET * deltaTheta;
        double theta = this.x.get(2);
        // Analytically solved jacobian
        return new SimpleMatrix(new double[][] {
                new double[] { 0, 0, -deltaX * Math.sin(theta) - deltaY * Math.cos(theta) },
                new double[] { 0, 0, deltaX * Math.cos(theta) - deltaY * Math.sin(theta) },
                new double[] { 0, 0, 0 }
        });
    }

    /**
     * Pretty much describes what the measurement should look like given the current state
     *
     * @return innovation estimate based on measurement vector
     */
    @Override
    public SimpleMatrix h() {
        return new SimpleMatrix(new DMatrixRMaj(distanceModule.getPredictedDistances(matrixToPose(this.x))));
    }

    /**
     * @return the jacobian of {@link #h} with respect to the state vector
     */
    @Override
    public SimpleMatrix H() {
        return distanceModule.jacobian(matrixToPose(this.x));
    }

    public Pose2d update() {
        return matrixToPose(super.iterate(new DMatrixRMaj(ArrayUtils.toPrimitive(Objects.requireNonNull(odometry.getWheelVelocities()).toArray(new Double[0]))), new DMatrixRMaj(distanceModule.getDistances())));
    }
}
