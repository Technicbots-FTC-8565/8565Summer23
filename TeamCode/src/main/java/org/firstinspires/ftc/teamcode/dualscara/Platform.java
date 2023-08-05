package org.firstinspires.ftc.teamcode.dualscara;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Platform {
    private DcMotorEx left, right;
    public static double UP_SPEED = 0.5;
    private Telemetry telemetry;
    public static double DOWN_SPEED = UP_SPEED;
    public static double CALIBRATING_SPEED = 0.15;
    public static int UP_POSITION = 50;
    public static int DOWN_POSITION = 0;
    public static int TOLERANCE = 5;

    public static PIDFCoefficients OPERATING_PID = new PIDFCoefficients(2.5, 0, 0.5, 5);
    public static PIDFCoefficients CALIBRATING_PID = new PIDFCoefficients(1.5, 0, 0, 2.5);
    public static double STALL_CURRENT = 1000;

    public final static int HOMING = 0;
    public final static int READY = 1;

    public int state;

    public Platform(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        left = hardwareMap.get(DcMotorEx.class, "leftPlatform");
        right = hardwareMap.get(DcMotorEx.class, "rightPlatform");
        telemetry = opMode.telemetry;
        state = HOMING;
        // left.setDirection(DcMotorSimple.Direction.REVERSE);
        initMotor(left);
        initMotor(right);
        home();
        down();
    }

    public void initMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CALIBRATING_PID);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, OPERATING_PID);
        motor.setTargetPositionTolerance(TOLERANCE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getPosition() {
        return (int) ((left.getCurrentPosition() + right.getCurrentPosition()) / 2);
    }

    public void setSpeed(double speed) {
        left.setPower(speed);
        right.setPower(speed);
    }

    public void stopCalibrating() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ready();
    }

    public void ready() {
        state = READY;
    }

    public void setPosition(int position) {
        if (position > getPosition()) setSpeed(UP_SPEED);
        else setSpeed(DOWN_SPEED);
        left.setTargetPosition(position);
        right.setTargetPosition(position);
    }

    public void inc(double amt) {
        this.setPosition(this.getPosition() + (int) amt);
    }

    public void down() {
        setPosition(DOWN_POSITION);
    }

    public void up() {
        setPosition(UP_POSITION);
    }

    public void home() {
        state = HOMING;
    }

    public void telemetryDebug() {
        telemetry.addData("Left Current", left.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Current", right.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Left Position", left.getCurrentPosition());
        telemetry.addData("Right Position", right.getCurrentPosition());
    }

    public void update() {
        if (state == HOMING) {
            if (left.getCurrent(CurrentUnit.MILLIAMPS) < STALL_CURRENT && right.getCurrent(CurrentUnit.MILLIAMPS) < STALL_CURRENT) {
                setSpeed(-CALIBRATING_SPEED);
                telemetryDebug();
            }
            else stopCalibrating();
        }
    }
}