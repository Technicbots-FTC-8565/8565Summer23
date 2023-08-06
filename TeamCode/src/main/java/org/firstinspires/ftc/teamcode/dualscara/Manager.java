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
    public static double PATHING_SPEED = 3.5;
    private Telemetry telemetry;
    private boolean running = false;
    public Manager(LinearOpMode opMode, List<List<Point>> segments, int[] route, double canvasWidth, double canvasHeight) {
        scara = new DualSCARA(opMode.hardwareMap, L0, A1, A2, B1, B2);
        platform = new Platform(opMode);
        this.segments = segments;
        this.route = route;
        this.queue = new ActionQueue();
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    public void start() {
        running = true;
        goNext();
    }

    public void goNext() {
        Point point = segments.get(route[currI]).get(currJ);
        double x = point.x + L0 / 2;
        double y = Range.clip(canvasHeight - point.y, 4, 10);
        telemetry.addData("Target x", x);
        telemetry.addData("Target y", y);
        Log.d("technicbots Target x", String.valueOf(x));
        Log.d("technicbots Target y", String.valueOf(y));
        if (scara.canMove(x, y)) {
            scara.x = x;
            scara.y = y;
        }
        else telemetry.addData("Error", "Target coords outside of workspace :(");
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
                running = false;
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
            if (segments.get(route[currI]).get(currJ).distanceTo(segments.get(route[cacheI]).get(cacheJ)) > 0.75) {
                telemetry.addData("Status", "Carrying out skipping");
                platform.down();
                queue.addDelayedAction(platform::up, segments.get(route[cacheI]).get(cacheJ).distanceTo(segments.get(route[currI]).get(currJ)) / PATHING_SPEED * 1000 - 75);
            }
            // path to new start of segment
            scara.x = segments.get(route[currI]).get(currJ).x;
            scara.y = segments.get(route[currI]).get(currJ).y;
            // append the next point to the pathing queue
            telemetry.addData("Cache i", cacheI);
            telemetry.addData("Cache j", cacheJ);
            Log.d("technicbots cache i", String.valueOf(cacheI));
            Log.d("technicbots new i", String.valueOf(currI));
            Log.d("technicbots cache j", String.valueOf(cacheJ));
            Log.d("technicbots new j", String.valueOf(currJ));
            telemetry.update();
            queue.addDelayedAction(this::goNext, segments.get(route[cacheI]).get(cacheJ).distanceTo(segments.get(route[currI]).get(currJ)) / PATHING_SPEED * 1000);
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

    public void update() {
        telemetry.addData("Status", "Pathing");
        telemetry.update();
        queue.update();
    }

    public boolean done() {
        return !running;
    }
}