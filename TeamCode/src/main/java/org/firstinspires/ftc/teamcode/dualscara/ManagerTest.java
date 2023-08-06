package org.firstinspires.ftc.teamcode.dualscara;

import static org.firstinspires.ftc.teamcode.dualscara.DualSCARATest.mmToInches;
import static org.firstinspires.ftc.teamcode.dualscara.util.Loader.load;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dualscara.util.GreedySalesman;
import org.firstinspires.ftc.teamcode.dualscara.util.Point;

import java.util.List;

@Config
@Autonomous(name="Manager Test", group="Testing")
public class ManagerTest extends LinearOpMode {
    private Manager manager;
    double CANVAS_WIDTH = mmToInches(250);
    double CANVAS_HEIGHT = mmToInches(250);
    private Telemetry dashboardTelemetry;
    @Override
    public void runOpMode() {
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        changeStatus("Loading polylines");
        List<List<Point>> segments = load(this, "tmp.csv", CANVAS_WIDTH, CANVAS_HEIGHT, 0.75);
        changeStatus("Successfully loaded polylines");
        changeStatus("Solving path segments");
        int[] route = GreedySalesman.solve(GreedySalesman.construct(segments));
        changeStatus("Successfully loaded greedy salesman");
        manager = new Manager(this, segments, route, CANVAS_WIDTH, CANVAS_HEIGHT);
        changeStatus("Calibrating platform");
        while (opModeInInit() && !opModeIsActive() && manager.platform.state == Platform.HOMING) {
            manager.platform.update(); // homing sequence runs
        }
        changeStatus("Waiting for start");
        waitForStart();
        manager.start();

        while (opModeIsActive() && !manager.done()) {
            manager.update();
        }

        changeStatus("Finished");
    }

    private void changeStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
        dashboardTelemetry.addData("Status", status);
        dashboardTelemetry.update();
    }
}
