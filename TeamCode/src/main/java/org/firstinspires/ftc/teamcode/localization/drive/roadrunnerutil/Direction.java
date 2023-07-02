package org.firstinspires.ftc.teamcode.localization.drive.roadrunnerutil;

public enum Direction {
	FORWARD(1),
	REVERSE(-1);

	public int multiplier;

	Direction(int multiplier) {
		this.multiplier = multiplier;
	}

	public int getMultiplier() {
		return multiplier;
	}
}
