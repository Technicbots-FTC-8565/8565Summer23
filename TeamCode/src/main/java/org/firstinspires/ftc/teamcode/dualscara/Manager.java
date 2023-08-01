package org.firstinspires.ftc.teamcode.dualscara;

import static org.firstinspires.ftc.teamcode.dualscara.DualSCARATest.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public static int PATHING_SPEED = 5;
    private Telemetry telemetry;
    public Manager(LinearOpMode opMode, List<List<Point>> segments, int[] route, double canvasWidth, double canvasHeight) {
        scara = new DualSCARA(opMode.hardwareMap, L0, A1, A2, B1, B2);
        platform = new Platform(opMode);
        this.segments = segments;
        this.route = route;
        this.queue = new ActionQueue();
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
        this.telemetry = opMode.telemetry;
    }

    public void start() {
        goNext();
    }

    public void goNext() {
        Point point = segments.get(route[currJ]).get(currJ);
        double x = point.x - (canvasWidth - L0 / 2);
        double y = point.y;
        telemetry.addData("Target x", x);
        telemetry.addData("Target y", y);
        if (scara.canMove(x, y)) {
            scara.x = x;
            scara.y = y;
            scara.update();
        }
        else telemetry.addData("Error", "Target coords outside of workspace :(");
        currJ += reverse? -1 : 1;
        // have we finished this path segment
        if ((!reverse && currJ > segments.get(route[currI]).size() - 1) || (reverse && currJ < 0)) {
            int cacheI = currI;
            int cacheJ = currJ - (reverse? -1 : 1);
            // ignore the last segment because it just loops back around
            if (currI > segments.size() - 2) {
                telemetry.addData("Status", "Finished Drawing");
                return; // return from fn without queueing the last segment
            }
            if (segments.get(route[currI]).get(segments.get(route[currI]).size() - 1)
                    .distanceTo(segments.get(route[currI + 1]).get(segments.get(route[currI + 1]).size() - 1)) <
                    segments.get(route[currI]).get(segments.get(route[currI]).size() - 1)
                    .distanceTo(segments.get(route[currI + 1]).get(0))) reverse = true;

            else reverse = false;
            currI++;
            if (reverse) currJ = segments.get(route[currI]).size() - 1;
            else currJ = 0;
            // allow 0.75 inch skipping
            if (segments.get(route[currI]).get(currJ).distanceTo(segments.get(route[cacheI]).get(cacheJ)) > 0.75) {
                platform.down();
                queue.addDelayedAction(platform::up, segments.get(route[cacheI]).get(cacheJ).distanceTo(segments.get(route[currI]).get(currJ)) / PATHING_SPEED - 0.075);
            }
            // path to new start of segment
            scara.x = segments.get(route[currI]).get(currJ).x;
            scara.y = segments.get(route[currI]).get(currJ).y;
            // append the next point to the pathing queue
            queue.addDelayedAction(this::goNext, segments.get(route[cacheI]).get(cacheJ).distanceTo(segments.get(route[currI]).get(currJ)) / PATHING_SPEED);
        }
        if ((!reverse && currJ < segments.get(route[currI]).size() - 1) || (reverse && currJ > 0)) {
            // append the next point to the pathing queue
            queue.addDelayedAction(this::goNext, segments.get(route[currI]).get(currJ).distanceTo(segments.get(route[currI]).get(currJ + (reverse? -1 : 1))) / PATHING_SPEED);
        }

        telemetry.update();
    }

    public void update() {
        telemetry.addData("Status", "Pathing");
        telemetry.update();
        queue.update();
    }
}