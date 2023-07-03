package org.firstinspires.ftc.teamcode.dualscara;

import static org.firstinspires.ftc.teamcode.dualscara.MathEx.square;

public class DualSCARA {
	private boolean shouldIk;
	private final double L0, A1, A2, B1, B2;
	private double T1, T2;
	private double workingMode1, workingMode2, fkMode;
	private double x, y, xCache, yCache;
	
	public DualSCARA(double L0, double A1, double A2, double B1, double B2) {
		this.L0 = L0;
		this.T1 = Math.PI * 0.6;
		this.T2 = Math.PI * 0.4;
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
		this.disableIk();
		this.forwardKinematics();
		this.enableIk();
	}

	public void update() {
		if (this.shouldIk) this.inverseKinematics();
		else this.forwardKinematics();
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
		double y_p = (-v4 + this.fkMode * Math.sqrt(square(v4) - 4 * v3 * v5)) / (2 * v3);
		double x_p = v1 * y_p + v2;
		this.x = x_p;
		this.y = y_p;
	}

	public void disableIk() {
		this.shouldIk = false;
	}

	public void enableIk() {
		this.shouldIk = true;
		this.x = xCache;
		this.y = yCache;
	}

	public void flipFk() {
		this.fkMode *= -1;
	}

	public void flipLeftWorkingMode() {
		this.workingMode1 *= -1;
	}

	public void flipRightWorkingMode() {
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
