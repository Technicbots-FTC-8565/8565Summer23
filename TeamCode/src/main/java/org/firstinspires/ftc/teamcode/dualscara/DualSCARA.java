package org.firstinspires.ftc.teamcode.dualscara;

import static org.firstinspires.ftc.teamcode.dualscara.MathEx.square;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DualSCARA {
	private boolean shouldIk;
	private final double L0, A1, A2, B1, B2;
	public double T1, T2, x, y;
	private double workingMode1, workingMode2, fkMode, xCache, yCache;
	private Servo left, right;
	public static double INIT_ANGLE = 0.5;
	public static double SERVO_ROM = Math.toRadians(270); // Range of motion of the servo, in radians
	public static double SERVO_LEFT_DEFAULT = 0.5; // To get an angle of 0 on L2
	public static double SERVO_RIGHT_DEFAULT = 0.5; // To get an angle of 0 on L1
	
	public DualSCARA(HardwareMap hardwareMap, double L0, double A1, double A2, double B1, double B2) {
		this.L0 = L0;
		this.T1 = Math.PI * (1d - INIT_ANGLE);
		this.T2 = Math.PI * INIT_ANGLE;
		this.A1 = A1;
		this.A2 = A2;
		this.B1 = B1;
		this.B2 = B2;
		this.x = 0;
		this.y = 0;
		this.xCache = 0;
		this.yCache = 0;
		this.workingMode1 = 1;
		this.workingMode2 = 1;
		this.fkMode = 1;
		this.left = hardwareMap.servo.get("leftSCARA");
		this.right = hardwareMap.servo.get("rightSCARA");
		this.disableIk();
		this.update(); // fk to set up x and y to the right values
		this.enableIk();
	}

	public void update() {
		if (this.shouldIk) this.inverseKinematics();
		else this.forwardKinematics();
		this.move();
	}

	public void move() {
		this.left.setPosition(SERVO_LEFT_DEFAULT + -(this.T1 - Math.PI) / SERVO_ROM);
		this.right.setPosition(SERVO_RIGHT_DEFAULT + -this.T2 / SERVO_ROM);
	}

	public void forwardKinematics() {
		double x_R1 = this.A1 * Math.cos(this.T1);
		double y_R1 = this.A1 * Math.sin(this.T1);
		double x_R2 = this.A2 * Math.cos(this.T2) + this.L0;
		double y_R2 = this.A2 * Math.sin(this.T2);

		double v1 = (y_R2 - y_R1) / (x_R1 - x_R2);
		double v2 = (square(this.B2) - square(this.B1) - square(y_R2) - square(x_R2) + square(y_R1) + square(x_R1)) / (2 * (x_R1 - x_R2));
		double v3 = 1 + square(v1);
		double v4 = 2 * (v1 * v2 - v1 * x_R1 - y_R1);
		double v5 = square(v2) + square(x_R1) - 2 * v2 * x_R1 + square(y_R1) - square(this.B1);
		this.y = (-v4 + this.fkMode * Math.sqrt(square(v4) - 4 * v3 * v5)) / (2 * v3);
		this.x = v1 * this.y + v2;
	}

	public void disableIk() {
		this.shouldIk = false;
	}

	public void enableIk() {
		this.shouldIk = true;
	}

	public boolean shouldIk() {
		return shouldIk;
	}

	public void toggleIk() {
		this.shouldIk = !this.shouldIk;
	}

	public void flipFk() {
		this.fkMode *= -1;
	}

	public void flipLeft() {
		this.workingMode1 *= -1;
	}

	public void flipRight() {
		this.workingMode2 *= -1;
	}

	/**
	 * Updates the arm's motor angles based on the desired end effector positions
	 */
	public void inverseKinematics() {
		double c1 = Math.sqrt(square(this.x) + square(this.y));
		double c2 = Math.sqrt(square(this.x - this.L0) + square(this.y));
		// stop the process if we have exceeded the workspace boundary
		if (Math.abs(this.x / c1) > 1 || Math.abs((this.L0 - this.x) / c2) > 1) return;
		double alpha_1 = Math.acos(this.x / c1);
		double alpha_2 = Math.acos((this.L0 - this.x) / c2);
		// stop the process if we have exceeded the workspace boundary
		if (Math.abs((square(c2) + square(this.A2) - square(this.B2)) / (2 * this.A2 * c2)) > 1 || Math.abs((square(c2) + square(this.A2) - square(this.B2)) / (2 * this.A2 * c2)) > 1) return;
		double beta_1 = this.workingMode1 * Math.acos((square(c1) + square(this.A1) - square(this.B1)) / (2 * this.A1 * c1));
		double beta_2 = this.workingMode2 * Math.acos((square(c2) + square(this.A2) - square(this.B2)) / (2 * this.A2 * c2));
		double theta_1 = alpha_1 + beta_1;
		double theta_2 = alpha_2 + beta_2;
		this.T1 = theta_1;
		this.T2 = Math.PI - theta_2;
	}

	public double getServoLeft() {
		return this.left.getPosition();
	}

	public double getServoRight() {
		return this.right.getPosition();
	}

	/**
	 * Checks to see if the specified coordinates are within the arm's workspace
	 * @param newX New x coordinate
	 * @param newY New y coordinate
	 * @return Whether or not the arm can move to those coordinates
	 */
	public boolean canMove(double newX, double newY) {
		double c1 = Math.sqrt(square(newX) + square(newY));
		double c2 = Math.sqrt(square(newX - this.L0) + square(newY));
		double alpha_1 = newX / c1;
		double alpha_2 = (this.L0 - newX) / c2;
		double beta_1 = (square(c1) + square(this.A1) - square(this.B1)) / (2 * this.A1 * c1);
		double beta_2 = (square(c2) + square(this.A2) - square(this.B2)) / (2 * this.A2 * c2);
		return Math.abs(alpha_1) <= 1 && Math.abs(alpha_2) <= 1 && Math.abs(beta_1) <= 1 && Math.abs(beta_2) <= 1;
	}
}
