package org.firstinspires.ftc.teamcode.dualscara.util;

import static org.firstinspires.ftc.teamcode.dualscara.MathEx.square;

public class Point {
    public int index;
    public double x, y;
    public Point(int index, double x, double y, double xOffset, double yOffset) {
        this.index = index;
        this.x = x + xOffset;
        this.y = y + yOffset;
    }

    public double distanceTo(Point other) {
        return Math.sqrt(square(other.y - this.y) + square(other.x - this.x));
    }
}
