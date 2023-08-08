package org.firstinspires.ftc.teamcode.dualscara;

import static org.firstinspires.ftc.teamcode.dualscara.DualSCARATest.*;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dualscara.util.Point;
import org.firstinspires.ftc.teamcode.util.ActionQueue;

import java.util.List;

@Config
public class Manager {
    public DualSCARA scara;
    public Platform platform;
    private List<List<Point>> segments;
    private int[] route;
    private boolean reverse = false;
    int currI = 0, currJ = 0;
    private ActionQueue queue;
    private double canvasWidth, canvasHeight;
    /** Pathing speed of the arm in inches per second */
    public static double PATHING_SPEED = 5;
    public static double MAX_SKIP = 0.25;
    private final Telemetry telemetry;
    private boolean running = false;
    private final double xOffset;
    private final double yOffset;
    public Manager(LinearOpMode opMode, List<List<Point>> segments, int[] route, double canvasWidth, double canvasHeight, double xOffset, double yOffset) {
        scara = new DualSCARA(opMode.hardwareMap, L0, A1, A2, B1, B2);
        platform = new Platform(opMode);
        this.segments = segments;
        Log.d("technicbots segments length debug", String.valueOf(segments.size()));
        this.route = route;
        this.queue = new ActionQueue();
        this.canvasWidth = canvasWidth;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.canvasHeight = canvasHeight;
        this.telemetry = FtcDashboard.getInstance().getTelemetry();

        // path to the first point if possible
        // moveTo(segments.get(0).get(0).x, segments.get(0).get(0).y);
        // this.queue.addDelayedAction(() -> this.queue.addDelayedAction(platform::up, 500), () -> platform.state == Platform.READY);
    }

    public void start() {
        running = true;
        goNext();
    }

    public void goNext() {
        Point point = segments.get(route[currI]).get(currJ);
        moveTo(point.x, point.y);
        scara.update();
        currJ += reverse? -1 : 1;
        Log.d("technicbots curri", String.valueOf(currI));
        Log.d("technicbots currj", String.valueOf(currJ));
        // have we finished this path segment
        if ((!reverse && currJ > segments.get(route[currI]).size() - 2) || (reverse && currJ < 1)) {
            int cacheI = currI;
            int cacheJ = currJ - (reverse? -1 : 1);
            // ignore the last segment because it just loops back around
            if (currI > segments.size() - 2) {
                telemetry.addData("Status", "Finished Drawing");
                telemetry.update();
                queue.addDelayedAction(() -> running = false, 250); // don't immediately stop
                return; // return from fn without queueing the last segment
            }
            reverse = segments.get(route[currI]).get(segments.get(route[currI]).size() - 1)
                    .distanceTo(segments.get(route[currI + 1]).get(segments.get(route[currI + 1]).size() - 1)) <
                    segments.get(route[currI]).get(segments.get(route[currI]).size() - 1)
                            .distanceTo(segments.get(route[currI + 1]).get(0));
            currI++;
            if (reverse) currJ = segments.get(route[currI]).size() - 1;
            else currJ = 0;
            // allow 0.75 inch skipping
            if (segments.get(route[currI]).get(currJ).distanceTo(segments.get(route[cacheI]).get(cacheJ)) > MAX_SKIP) {
                telemetry.addData("Status", "Carrying out skipping");
                // platform.down();
                // queue.addDelayedAction(platform::up, segments.get(route[cacheI]).get(cacheJ).distanceTo(segments.get(route[currI]).get(currJ)) / PATHING_SPEED * 1000 + 1000);
            }
            // path to new start of segment
            queue.addDelayedAction(() -> moveTo(segments.get(route[currI]).get(currJ).x, segments.get(route[currI]).get(currJ).y), 300);
            // append the next point to the pathing queue
            telemetry.addData("Cache i", cacheI);
            telemetry.addData("Cache j", cacheJ);
            Log.d("technicbots cache i", String.valueOf(cacheI));
            Log.d("technicbots new i", String.valueOf(currI));
            Log.d("technicbots cache j", String.valueOf(cacheJ));
            Log.d("technicbots new j", String.valueOf(currJ));
            telemetry.update();
            queue.addDelayedAction(this::goNext, segments.get(route[cacheI]).get(cacheJ).distanceTo(segments.get(route[currI]).get(currJ)) / PATHING_SPEED * 1000); // + 500);
        }
        else if ((!reverse && currJ < segments.get(route[currI]).size() - 1) || (reverse && currJ > 0)) {
            // append the next point to the pathing queue
            queue.addDelayedAction(this::goNext, segments.get(route[currI]).get(currJ).distanceTo(segments.get(route[currI]).get(currJ + (reverse? -1 : 1))) / PATHING_SPEED * 1000);
            telemetry.addData("Distance to next point", segments.get(route[currI]).get(currJ).distanceTo(segments.get(route[currI]).get(currJ + (reverse? 1 : -1))));
        }
        telemetry.addData("Current Path Segment", currI);
        telemetry.addData("Current Path Point", currJ);

        telemetry.update();
    }

    public void moveTo(double x, double y) {
        double modX = Range.clip(x + L0 / 2 + xOffset, -4, 14);
        double modY = Range.clip(canvasHeight - y + yOffset, 3, 11);
        telemetry.addData("Target x", x);
        telemetry.addData("Target y", y);
        Log.d("technicbots Target x", String.valueOf(x));
        Log.d("technicbots Target y", String.valueOf(y));
        if (scara.canMove(modX, modY)) {
            scara.x = modX;
            scara.y = modY;
            scara.update();
        }
        else telemetry.addData("Error", "Target coords outside of workspace :(");
    }

    public void update() {
        telemetry.addData("Status", "Pathing");
        telemetry.update();
        queue.update();
        // platform.update();
    }

    public boolean done() {
        return !running;
    }
}