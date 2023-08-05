package org.firstinspires.ftc.teamcode.dualscara.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

@Config
public class Loader {
    public static double SCALING_FACTOR = 1.5;
    public static double ORIG_WIDTH = 700 * SCALING_FACTOR;
    public static double ORIG_HEIGHT = 631.7796610169491 * SCALING_FACTOR;
    /**
     * Load a CSV to parsed points
     * @param filename CSV location relative to project's "Assets" folder
     * @param canvasWidth Width of the IRL canvas in IRL units
     * @param canvasHeight Height of the IRL canvas in IRL units
     * @param scaling Factor to scale the image by
     * @return The parsed points from the CSV file
     */
    public static List<List<Point>> load(OpMode opMode, String filename, double canvasWidth, double canvasHeight, double scaling) {
        try (BufferedReader br = new BufferedReader(new InputStreamReader(opMode.hardwareMap.appContext.getAssets().open(filename)))) {
            double scalingFactor = Math.min(canvasWidth / ORIG_WIDTH, canvasHeight / ORIG_HEIGHT) * scaling;
            List<List<Point>> ret = new ArrayList<>();
            List<Point> pointBuffer = new ArrayList<>();
            int prevIndex = 0;
            String line;
            while ((line = br.readLine()) != null) {
                String[] sv = line.split(",");
                if (prevIndex != Integer.parseInt(sv[0])) {
                    if (pointBuffer.size() > 5) ret.add(new ArrayList<>(pointBuffer)); // ignore tiny polylines
                    pointBuffer.clear();
                }
                Point constrPoint = new Point(Integer.parseInt(sv[0]), Double.parseDouble(sv[1]) * scalingFactor, Double.parseDouble(sv[2]) * scalingFactor, (canvasWidth - canvasWidth / scaling) / 2, (canvasHeight - canvasHeight / scaling) / 2);
                pointBuffer.add(constrPoint);
                prevIndex = constrPoint.index;
            }
            return ret;
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
