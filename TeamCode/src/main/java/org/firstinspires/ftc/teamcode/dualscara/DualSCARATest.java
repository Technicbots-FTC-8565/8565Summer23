package org.firstinspires.ftc.teamcode.dualscara;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Dual SCARA Test", group = "Testing")
public class DualSCARATest extends OpMode {
    private DualSCARA scara;
    private GamepadEx gpad1;
    public static double SENS = 0.05;
    public static double L0 = mmToInches(192);
    public static double A1 = mmToInches(108);
    public static double A2 = A1;
    public static double B1 = mmToInches(180);
    public static double B2 = B1;

    public static double mmToInches(double mm) {
        return mm * 0.03937007874;
    }

    @Override
    public void init() {
        scara = new DualSCARA(hardwareMap, L0, A1, A2, B1, B2);
        gpad1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gpad1.readButtons();
        if (gpad1.wasJustPressed(GamepadKeys.Button.A)) scara.toggleIk();
        else if (gpad1.wasJustPressed(GamepadKeys.Button.X)) scara.flipLeft();
        else if (gpad1.wasJustPressed(GamepadKeys.Button.B)) scara.flipRight();
        else if (gpad1.wasJustPressed(GamepadKeys.Button.Y)) scara.flipFk();
        if (scara.shouldIk()) {
            double dx = SENS * gpad1.getLeftX();
            double dy = SENS * gpad1.getRightY();
            if (scara.canMove(scara.x + dx, scara.y + dy)) {
                scara.x += dx;
                scara.y += dy;
            }
        }
        else {
            double d1 = gpad1.getLeftX() * SENS / 40;
            double d2 = gpad1.getRightX() * SENS / 40;
            scara.T1 += d1;
            scara.T2 += d2;
        }

        scara.update();
        telemetry.addData("T1", scara.T1);
        telemetry.addData("T2", scara.T2);
        telemetry.addData("Servo Left", scara.getServoLeft());
        telemetry.addData("Servo Right", scara.getServoRight());
    }
}