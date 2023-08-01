package org.firstinspires.ftc.teamcode.dualscara.util;

import java.util.Arrays;
import java.util.List;

public class GreedySalesman {
    /**
     * @param distances Adjacency matrix representing distance between each node
     * @return Solved index order
     */
    public static int[] solve(double[][] distances) {
        double min = 1e9;
        int counter = 0, j = 0, i = 0;
        int[] route = new int[distances.length + 1];
        int[] visitedRouteList = new int[distances.length];
        
        Arrays.fill(visitedRouteList, 0);
        visitedRouteList[0] = 1; // start from 1st node
        route[0] = 0; // start from 1st node

        // Traverse the adjacency matrix
        while (i < distances.length && j < distances[i].length) {
            // Corner of the Matrix
            if (counter >= distances[i].length - 1) break;

            // Update cost
            if (j != i && (visitedRouteList[j] == 0)) {
                if (distances[i][j] < min) {
                    min = distances[i][j];
                    route[counter + 1] = j + 1;
                }
            }
            j++;

            if (j == distances[i].length) {
                min = 1e9;
                visitedRouteList[route[counter + 1] - 1] = 1;
                j = 0;
                i = route[counter + 1] - 1;
                counter++;
            }
        }

        i = route[counter - 1] - 1;

        for (j = 0; j < distances.length; j++) {
            if ((i != j) && distances[i][j] < min) {
                min = distances[i][j];
                route[counter + 1] = j + 1;
            }
        }

        for (int c : route) route[c]--;

        int[] parsedRoute = new int[route.length - 1];
        // basically just popping the last int
        System.arraycopy(route, 0, parsedRoute, 0, parsedRoute.length);

        return parsedRoute;
    }

    /**
     * @param segments Segments parsed from polyline CSV
     * @return Adjacency matrix constructed from segments
     */
    public static double[][] construct(List<List<Point>> segments) {
        double[][] ret = new double[segments.size()][segments.size()];
        for (int i = 0; i < segments.size(); i++) {
            Point endPoint = segments.get(i).get(segments.get(i).size() - 1);
            for (int j = 0; j < segments.size(); j++) {
                if (i == j) ret[i][j] = -1;
                else ret[i][j] = Math.min(endPoint.distanceTo(segments.get(j).get(0)), endPoint.distanceTo(segments.get(j).get(segments.get(j).size() - 1)));
            }
        }

        return ret;
    }
}
