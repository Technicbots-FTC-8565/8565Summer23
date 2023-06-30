package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

public class EjmlTest {
    public static void main(String[] args) {
        Pose2d pose = new Pose2d(4, 5, 6);
        DMatrixRMaj maj = new DMatrixRMaj(new double[] { pose.getX(), pose.getY(), pose.getHeading() });
        SimpleMatrix matrix = new SimpleMatrix(maj);
        SimpleMatrix testCov = new SimpleMatrix(new double[][] {
                new double[] { 1, 2, 3 },
                new double[] { 1, 2, 3 },
                new double[] { 1, 2, 3 }
        });
        System.out.println(pose);
        System.out.println(maj);
        System.out.println(testCov);
        System.out.println(testCov.mult(matrix));
        System.exit(0);
    }
}
