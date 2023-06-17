package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="Empty OpMode", group="Testing")
public class EmptyOpMode extends OpMode {
    public void init() {
        telemetry.addData("Status", "Initialized");
    }

    public void start() {
        telemetry.addData("Status", "Started");
    }

    public void loop() {
        telemetry.addData("Status", "Looping");
    }

    public void stop() {
        telemetry.addData("Status", "Stopped");
    }
}
