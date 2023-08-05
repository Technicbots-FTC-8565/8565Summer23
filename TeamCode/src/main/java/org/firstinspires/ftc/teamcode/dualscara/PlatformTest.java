package org.firstinspires.ftc.teamcode.dualscara;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Platform Test", group = "Testing")
public class PlatformTest extends OpMode {
	private Platform platform;
	private GamepadEx gpad1;
	public static double SENS = 0.00001;

	@Override
	public void init() {
		platform = new Platform(this);
		gpad1 = new GamepadEx(gamepad1);
	}

	@Override
	public void init_loop() {
		platform.update(); // home
	}

	@Override
	public void loop() {
		gpad1.readButtons();
		if (gpad1.wasJustPressed(GamepadKeys.Button.B)) platform.home();
		else if (gpad1.wasJustPressed(GamepadKeys.Button.X)) platform.stopCalibrating(); // failsafe
		else if (gpad1.wasJustPressed(GamepadKeys.Button.A)) platform.down();
		else if (gpad1.wasJustPressed(GamepadKeys.Button.Y)) platform.up();
		if (Math.abs(gpad1.getLeftY()) > 0.05) platform.inc(Range.clip(SENS * gpad1.getLeftY(), -10, 10));

		platform.update();
		platform.telemetryDebug();
	}
}