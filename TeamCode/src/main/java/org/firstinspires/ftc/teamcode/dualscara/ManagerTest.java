package org.firstinspires.ftc.teamcode.dualscara;

import static org.firstinspires.ftc.teamcode.dualscara.util.Loader.load;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dualscara.util.GreedySalesman;
import org.firstinspires.ftc.teamcode.dualscara.util.Point;

import java.util.List;

@Config
public class ManagerTest extends LinearOpMode {
    private Manager manager;
    double CANVAS_WIDTH = 336;
    double CANVAS_HEIGHT = 256;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Loading polylines");
        telemetry.update();
        List<List<Point>> segments = load("calligraphy.csv", CANVAS_WIDTH, CANVAS_HEIGHT, 1);
        telemetry.addData("Status", "Finished loading polylines");
        telemetry.update();
        telemetry.addData("Status", "Loading path segments");
        telemetry.update();
        int[] route = GreedySalesman.solve(GreedySalesman.construct(segments));
        telemetry.addData("Status", "Finished loading path segments");
        telemetry.update();
        manager = new Manager(this, segments, route, CANVAS_WIDTH, CANVAS_HEIGHT);

        telemetry.addData("Status", "Calibrating platform");
        telemetry.update();
        while (opModeInInit() && !opModeIsActive()) {
            manager.platform.update(); // homing sequence runs
        }
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();
        manager.start();

        while (opModeIsActive()) manager.update();

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
