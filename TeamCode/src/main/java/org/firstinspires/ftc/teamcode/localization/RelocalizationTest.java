package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.drive.MecanumDriveCancellablev3;

@TeleOp(name = "Relocalization Test", group = "Testing")
public class RelocalizationTest extends OpMode {
    private MecanumDriveCancellablev3 chassis;
    @Override
    public void init() {
        chassis = new MecanumDriveCancellablev3(hardwareMap);
    }

    @Override
    public void loop() {
        chassis.update();
    }
}
